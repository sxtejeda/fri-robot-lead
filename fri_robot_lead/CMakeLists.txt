cmake_minimum_required(VERSION 2.8.3)
project(fri_robot_lead)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
	sensor_msgs
	cv_bridge
	image_transport
	message_generation
	bwi_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	FILES
	PersonPresent.msg	
)

generate_messages(
	DEPENDENCIES
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	DEPENDS
	bwi_msgs
)

# Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}er src/leader.cpp)

 add_executable(person_detector src/person_detector.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}er
   ${catkin_LIBRARIES}
 )

target_link_libraries(person_detector
	${catkin_LIBRARIES}
)


