cmake_minimum_required(VERSION 2.8.3)
project(ur_kdl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  orocos_kdl
  message_generation
)

add_service_files(
  FILES
  IK.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur_kdl src/ur_kdl.cpp src/ur_kdl_node.cpp)
target_link_libraries(ur_kdl ${catkin_LIBRARIES})
add_dependencies(ur_kdl ${ur_kdl_EXPORTED_TARGETS})