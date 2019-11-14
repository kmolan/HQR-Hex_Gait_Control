cmake_minimum_required(VERSION 2.8.3)
project(pose_info)
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
)
find_package(Eigen3 REQUIRED)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_info src/pose_info.cpp)

target_link_libraries(pose_info
  ${catkin_LIBRARIES}
)
