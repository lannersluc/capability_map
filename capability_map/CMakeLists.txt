cmake_minimum_required(VERSION 2.8.3)
project(capability_map)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
  message_generation
)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  Reachability_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES capability_octree
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs
  DEPENDS octomap 
)

include_directories(include)
include_directories(
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(capability_octree src/CapabilityOcTreeNode.cpp src/CapabilityOcTree.cpp)
target_link_libraries(capability_octree ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(CapabilityTest src/CapabilityTest.cpp)
target_link_libraries(CapabilityTest capability_octree ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(capability_map src/main.cpp)
target_link_libraries(capability_map capability_octree ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(capability_map capability_map_generate_messages_cpp)

add_executable(capability_vis src/CapabilityVis.cpp)
target_link_libraries(capability_vis capability_octree ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})

