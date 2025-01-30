cmake_minimum_required(VERSION 3.5)
project(bspline_optimization)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  rospy
  tf
)

find_package( 
  Eigen3
  REQUIRED
)

find_package(
  Ceres
  REQUIRED
)
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

include_directories(${catkin_INCLUDE_DIRS} include ${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS})

add_executable(bspline_optimization_node
  src/bspline_optimization_node.cpp
  src/bspline_optimization.cpp
)

target_link_libraries(bspline_optimization_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${NLOPT_LIBRARIES}
)
catkin_package()