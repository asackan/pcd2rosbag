cmake_minimum_required(VERSION 2.8.3)
project(make_rosbag)

set(CMAKE_CXX_FLAGS "-std=c++17 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS
roscpp rosbag sensor_msgs
pcl_ros pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbag toRosbag.cpp)
target_link_libraries(rosbag ${catkin_LIBRARIES})