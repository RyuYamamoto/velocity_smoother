cmake_minimum_required(VERSION 3.0.2)
project(velocity_smoother)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES velocity_smoother
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_smoother_node src/velocity_smoother_node.cpp src/velocity_smoother.cpp)
target_link_libraries(velocity_smoother_node ${catkin_LIBRARIES})
