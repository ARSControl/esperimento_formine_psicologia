cmake_minimum_required(VERSION 3.0.2)
project(driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  
  # ur_kdl
  ur_rtde_controller
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs#   trajectory_msgs
# )

catkin_package(
  INCLUDE_DIRS 
  include
  #  LIBRARIES example_package
  CATKIN_DEPENDS
  message_runtime
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(driver src/drivejoints_node.cpp src/drivejoints.cpp)
# target_link_libraries(driver ${catkin_LIBRARIES})
# add_dependencies(driver ${catkin_EXPORTED_TARGETS})

install(PROGRAMS 
  scripts/driver.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
