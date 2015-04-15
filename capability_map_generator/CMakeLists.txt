cmake_minimum_required(VERSION 2.8.3)
project(capability_map_generator)

find_package(catkin REQUIRED COMPONENTS
  capability_map
  pluginlib
  roscpp
  visualization_msgs
  eigen
  moveit_core
  moveit_ros_planning
)
find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES reachability_sphere reachability_interface reachability_dummy_interface reachability_ros_kinematics_interface
    CATKIN_DEPENDS capability_map pluginlib roscpp visualization_msgs
    DEPENDS eigen
)

include_directories(include ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(reachability_sphere src/ReachabilitySphere.cpp)

add_library(reachability_interface src/ReachabilityInterface.cpp)

catkin_add_gtest(CapabilityMapGeneratorTest src/CapabilityMapGeneratorTest.cpp)
target_link_libraries(CapabilityMapGeneratorTest reachability_sphere ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

add_executable(capability_map_generator src/main.cpp src/ReachabilitySphere.cpp)
target_link_libraries(capability_map_generator reachability_interface ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

add_executable(reachability_vis src/ReachabilityVis.cpp src/ReachabilitySphere.cpp)
target_link_libraries(reachability_vis reachability_interface ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

add_executable(merge_capability_maps src/MergeCapabilityMaps.cpp)
target_link_libraries(merge_capability_maps ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

add_library(reachability_dummy_interface src/ReachabilityDummyInterface.cpp)
target_link_libraries(reachability_dummy_interface reachability_interface ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

add_library(reachability_ros_kinematics_interface src/ReachabilityROSKinematicsInterface.cpp)
target_link_libraries(reachability_ros_kinematics_interface reachability_interface ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

