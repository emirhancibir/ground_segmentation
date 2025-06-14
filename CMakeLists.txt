cmake_minimum_required(VERSION 3.0.2)
project(ground_segmentation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ground_segmentation_node src/ground_segmentation.cpp)

target_link_libraries(ground_segmentation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
