cmake_minimum_required(VERSION 3.16)
project(detector)

#set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  aruco_msgs
  geometry_msgs
  mavros
  roscpp
  tf
  prometheus_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_detector
#  CATKIN_DEPENDS geometry_msgs mavros roscpp
#  DEPENDS system_lib
)
#link_directories("/usr/local/lib")
#include_directories("/usr/local/include")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(detector_vel src/detector_vel.cpp src/Fuzzy_PID.cpp src/Incremental_PID.cpp)

add_executable(simulation_detector src/simulation_detector.cpp src/mpc.cpp src/Fuzzy_PID.cpp)
target_link_libraries(simulation_detector ${catkin_LIBRARIES} ipopt casadi)
#target_link_libraries(simulation_detector ${catkin_LIBRARIES} osqp OsqpEigen)

add_executable(simulation_detector_landing src/simulation_detector_landing.cpp src/mpc_landing.cpp)
target_link_libraries(simulation_detector_landing ${catkin_LIBRARIES} ipopt casadi)

add_executable(simulation_detector_move_landing src/simulation_detector_move_landing.cpp src/mpc_move_landing.cpp)
target_link_libraries(simulation_detector_move_landing ${catkin_LIBRARIES} ipopt casadi)

add_executable(simulation_detector_move_curve_landing src/simulation_detector_move_curve_landing.cpp src/mpc_move_landing.cpp)
target_link_libraries(simulation_detector_move_curve_landing ${catkin_LIBRARIES} ipopt casadi)

add_executable(forward_delay_node src/forward_delay_node.cpp)
target_link_libraries(forward_delay_node ${catkin_LIBRARIES})

add_executable(feedback_delay_node src/feedback_delay_node.cpp)
target_link_libraries(feedback_delay_node ${catkin_LIBRARIES})


