cmake_minimum_required(VERSION 3.0.2)
project(arduinocontroller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
#   std_msgs
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduinocontroller src/ardcon_node.cpp src/ardcon.cpp)
target_link_libraries(arduinocontroller ${catkin_LIBRARIES})
add_dependencies(arduinocontroller ${catkin_EXPORTED_TARGETS})
