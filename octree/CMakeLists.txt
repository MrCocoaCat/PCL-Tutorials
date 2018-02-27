cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(octree_change_detection)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (point_cloud_compression point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})

add_executable (octree_change_detection octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})


add_executable (octree_search octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES})
