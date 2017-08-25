cmake_minimum_required(VERSION 2.8.3)
project(rmi_driver)

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
  pluginlib 
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
        pluginlib 
    DEPENDS Boost
    LIBRARIES rmi_driver
    INCLUDE_DIRS include
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


#file(GLOB_RECURSE rmi_driver_src RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
set(SRC_FILES src/commands.cpp
              src/connector.cpp
              src/driver.cpp
              src/util.cpp
  )

add_library(rmi_driver ${SRC_FILES})
target_link_libraries(rmi_driver ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_executable(rmi_driver_node
  src/main.cpp
)

# add_dependencies(rmi_driver 
#   sensor_msgs_gencpp
#   ${PROJECT_NAME}_gencfg
#   ${PROJECT_NAME}_gencpp
# )

target_link_libraries(rmi_driver_node
  rmi_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_library(rmi_driver_lib src/commands.cpp)
#target_link_libraries(rmi_driver_lib ${catkin_LIBRARIES})




## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

