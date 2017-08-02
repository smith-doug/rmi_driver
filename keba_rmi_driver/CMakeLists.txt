cmake_minimum_required(VERSION 2.8.3)
project(keba_rmi_driver)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	geometry_msgs
	trajectory_msgs
	roscpp
	rospy
	sensor_msgs
	tf
	message_generation
	control_msgs
  iiwa_driver
	robot_movement_interface
  industrial_utils
)

find_package(Boost REQUIRED COMPONENTS
	system 
	thread)

## Set to debug compiler mode
SET(CMAKE_BUILD_TYPE debug)

#add_action_files(
#   FILES   
#)

# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   std_msgs
#)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        actionlib
        actionlib_msgs
        geometry_msgs
        trajectory_msgs
        roscpp
        rospy
        sensor_msgs
        tf
        iiwa_driver
		    robot_movement_interface
        industrial_utils
    DEPENDS Boost
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE keba_rmi_driver_src RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(keba_rmi_driver
  ${keba_rmi_driver_src}
)

add_dependencies(keba_rmi_driver 
  sensor_msgs_gencpp
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(keba_rmi_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
