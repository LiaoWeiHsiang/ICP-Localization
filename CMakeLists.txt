cmake_minimum_required(VERSION 2.8.3)
project(icp_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw4_0860915
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ITRI_localization src/ITRI_localization.cpp)
target_link_libraries(ITRI_localization ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ITRI_localization ${visualizer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(nuscenes_localization src/nuscenes_localization.cpp)
target_link_libraries(nuscenes_localization ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(nuscenes_localization ${visualizer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
