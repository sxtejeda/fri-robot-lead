import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class GuidePlugin(Plugin):

		def __init__(self, context):
			super (GuidePlugin, self).__init__(context)

			self.setObjectName('GuidePlugin')

			from argparse import ArgumentParser
			parser = ArgumentParser()

			parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")

			args, unknowns = parser.parse_known_args(context.argv())
			if not args.quiet:
				print 'arguments: ', args
				print 'unknowns: ', unknowns

			self._widget = QWidget()

			ui_file = os.path.join(rospkg.RosPack().get_path('new_rqt'), 'resource', 'GuidePlugin.ui')

			loadUi(ui_file, self._widget)

			self._widget.setObjectName('GuidePluginUi')

			self._widget.setWindowTitle('Title')

			context.add_widget(self._widget)

		def shutdown_plugin(self):
			pass

		def save_settings(self, plugin_settings, instance_settings):
			pass

		def restore_settings(self, plugin_settings, instance_settings):
			pass

	
	
