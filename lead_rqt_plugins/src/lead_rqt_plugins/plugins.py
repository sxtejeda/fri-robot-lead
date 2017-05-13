## NOTE this file copies and modifies bwi_common/bwi_rqt_plugins/src/bwi_rqt_plugs/plugins.py
## Modifications by Luke Wright 2017 for use in the robot leading functionality 
## Modifications provided to implement ability to allow a dropdown to select room to nav to

# SimpleRobotSteeringPlugin plugin based on rqt_robot_steering. Original
# copyright notice below.
#
# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# from __future__ import division

#import math
import rospy
import time

#from bwi_msgs.srv import QuestionDialog, QuestionDialogResponse, \
#                         QuestionDialogRequest
# modified .srv file defining the msgs 
from lead_rqt_plugins.srv import RoomDialog, RoomDialogResponse, RoomDialogRequest

from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding.QtGui import QFont, QGridLayout, QLabel, QLineEdit, \
                                    QPushButton, QTextBrowser, QVBoxLayout, QWidget, QComboBox, \
                                    QHBoxLayout

import os
import rospkg

# from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, SIGNAL, Slot

class RoomDialogPlugin(Plugin):

    def __init__(self, context):
        super(RoomDialogPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RoomDialogPlugin')

        font_size = rospy.get_param("~font_size", 30)

        # Create QWidget
        self._widget = QWidget()
        self._widget.setFont(QFont("Times", font_size, QFont.Bold))
        self._layout = QVBoxLayout(self._widget)
        self._text_browser = QTextBrowser(self._widget)
        self._layout.addWidget(self._text_browser)
        self._button_layout = QGridLayout()
        self._layout.addLayout(self._button_layout)
        rospy.loginfo("Hello world")

        # Add combobox
        self._cb_layout = QHBoxLayout()
        self._cb = QComboBox()
        self._layout.addLayout(self._cb_layout)

        #layout = QVBoxLayout(self._widget)
        #layout.addWidget(self._button)
        self._widget.setObjectName('RoomDialogPluginUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Setup service provider
        self.service = rospy.Service('room_dialog', RoomDialog,
                                     self.service_callback)
        self.response_ready = False
        self.response = None
        self.buttons = []
        self.text_label = None
        self.text_input = None
        
        # Add combo options 
        
    
        

        self.connect(self._widget, SIGNAL("update"), self.update)
        self.connect(self._widget, SIGNAL("timeout"), self.timeout)

    def shutdown_plugin(self):
        self.service.shutdown()

    def service_callback(self, req):
        self.response_ready = False
        self.request = req
        self._widget.emit(SIGNAL("update"))
        # Start timer against wall clock here instead of the ros clock.
        start_time = time.time()
        while not self.response_ready:
            if self.request != req:
                # The request got preempted by a new request.
                return RoomDialogResponse(RoomDialogRequest.PREEMPTED, "")
            if req.timeout != RoomDialogRequest.NO_TIMEOUT:
                current_time = time.time()
                if current_time - start_time > req.timeout:
                    self._widget.emit(SIGNAL("timeout"))
                    return RoomDialogResponse(
                            RoomDialogRequest.TIMED_OUT, "")
            time.sleep(0.2)
        return self.response

    def update(self):
        self.clean()
        req = self.request
        self._text_browser.setText(req.message)
        if req.type == RoomDialogRequest.DISPLAY:
            # All done, nothing more too see here.
            self.response = RoomDialogResponse(
                    RoomDialogRequest.NO_RESPONSE, "")
            self.response_ready = True
        elif req.type == RoomDialogRequest.CHOICE_QUESTION:
            for index, options in enumerate(req.options):
                button = QPushButton(options, self._widget)
                button.clicked.connect(partial(self.handle_button, index))
                row = index / 3
                col = index % 3
                self._button_layout.addWidget(button, row, col)
                self.buttons.append(button)
        elif req.type == RoomDialogRequest.TEXT_QUESTION:
            self.text_label = QLabel("Enter here: ", self._widget)
            self._button_layout.addWidget(self.text_label, 0, 0)
            self.text_input = QLineEdit(self._widget)
            self.text_input.editingFinished.connect(self.handle_text)
            self._button_layout.addWidget(self.text_input, 0, 1)
        
        # add handling of combobox
        elif req.type == RoomDialogRequest.COMBOBOX_QUESTION:
            rospy.loginfo("Combobox selected")
            for index, options in enumerate(req.options):
                self._cb.addItem(options) 
                rospy.loginfo(options)
                #self.buttons.append(options)
            # NOTE COULD INTRODUCE BUG
            self._cb.currentIndexChanged.connect(self.handle_cb)
            self._cb_layout.addWidget(self._cb)

    def timeout(self):
        self._text_browser.setText("Oh no! The request timed out.")
        self.clean()

    def clean(self):
        while self._button_layout.count():
            item = self._button_layout.takeAt(0)
            item.widget().deleteLater()
        self.buttons = []
        self.text_input = None
        self.text_label = None

    def handle_button(self, index):
        self.response = RoomDialogResponse(index, "")
        self.clean()
        self.response_ready = True

    def handle_text(self):
        self.response = RoomDialogResponse(
            RoomDialogRequest.TEXT_RESPONSE,
            self.text_input.text())
        self.clean()
        self.response_ready = True
    
    def handle_cb(self, index):
        # This will be the sign format seen around building ex: 3.404
        rospy.loginfo("handling cb")
        roomHuman = self._cb.currentText()
        # modify string into robot format ex: d3_404
        splitHuman = roomHuman.split('.', 1)
        roomRobot = 'd' + splitHuman[0] + '_' + splitHuman[1]
        roomRobot = str(roomRobot)
        self.response = RoomDialogResponse(
            RoomDialogRequest.CB_RESPONSE,
            roomRobot
            )
        self.clean()
        self.response_ready = True

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

