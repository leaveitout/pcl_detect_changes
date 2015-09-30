cmake_minimum_required(VERSION 3.3)

project(pcl_detect_changes)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES main.cpp)

add_executable(pcl_detect_changes ${SOURCE_FILES})

target_link_libraries(pcl_detect_changes ${PCL_LIBRARIES})
target_link_libraries(pcl_detect_changes ${Boost_LIBRARIES})
target_link_libraries(pcl_detect_changes Qt5::Widgets)