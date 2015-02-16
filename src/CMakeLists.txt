cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_zoom_simulation)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
add_executable (pcl_visualizer_ht pcl_visualizer_ht.cpp triangulate.cpp parse.cpp)
target_link_libraries (pcl_visualizer_ht ${PCL_LIBRARIES})
