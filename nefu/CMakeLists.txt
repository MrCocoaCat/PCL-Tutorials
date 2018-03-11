cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(nefu)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (filter extract_indices.cpp )
target_link_libraries (filter ${PCL_LIBRARIES})


add_executable (surface  surface.cpp )
target_link_libraries (surface ${PCL_LIBRARIES})



add_executable (possion   possion.cpp)
target_link_libraries (possion ${PCL_LIBRARIES})


add_executable (etra   etra.cpp)
target_link_libraries (etra ${PCL_LIBRARIES})