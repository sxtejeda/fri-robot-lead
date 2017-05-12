cmake_minimum_required(VERSION 2.8.3)
project(lead_rqt_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  bwi_msgs
  class_loader
  cv_bridge
  image_transport
  rqt_gui
  rqt_gui_cpp
  sensor_msgs)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)


set(bwi_rqt_plugins_INCLUDE_DIRECTORIES
#  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_python_setup()
add_service_files(
  DIRECTORY srv
  FILES
    RoomDialog.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    bwi_msgs
    class_loader
    cv_bridge
    image_transport
    rqt_gui
    rqt_gui_cpp
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


include(${QT_USE_FILE})


include_directories(${bwi_rqt_plugins_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/image_view.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES python_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/room_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

