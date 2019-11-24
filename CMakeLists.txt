cmake_minimum_required(VERSION 2.8.3)
project(hqrhex_control)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
    FILES
    pose_info_msg.msg
    internal_states_msg.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  include
  msg
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_info src/pose_info.cpp)
add_executable(internal_states src/internal_states.cpp)

target_link_libraries(pose_info
  ${catkin_LIBRARIES}
)

target_link_libraries(internal_states
    ${catkin_LIBRARIES}
)

add_dependencies(pose_info hqrhex_control_generate_messages_cpp)
add_dependencies(internal_states hqrhex_control_generate_messages_cpp)