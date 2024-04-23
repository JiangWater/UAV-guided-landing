cmake_minimum_required(VERSION 3.0.2)
project(uav_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  message_generation
  nav_msgs
  prometheus_msgs
  roscpp
  rospy
  sensor_msgs
  aruco_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(uav_auto_takeoff src/uav_auto_takeoff.cpp)
target_link_libraries(uav_auto_takeoff ${catkin_LIBRARIES})

add_executable(uav_auto_takeoff_with_delay src/uav_auto_takeoff_with_delay.cpp)
target_link_libraries(uav_auto_takeoff_with_delay ${catkin_LIBRARIES})



