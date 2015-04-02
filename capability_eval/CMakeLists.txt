cmake_minimum_required(VERSION 2.8.3)
project(capability_eval)

find_package(catkin REQUIRED COMPONENTS
  capability_map
  capability_map_generator
  pluginlib
  roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(capability_eval src/CapabilityEvaluation.cpp)
target_link_libraries(capability_eval ${catkin_LIBRARIES})

