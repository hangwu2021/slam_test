cmake_minimum_required(VERSION 3.0)

project(ch8)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ch8 ./8-6-1-many_feature_detect_ways_with_comparing.cpp)

target_link_libraries(ch8 ${OpenCV_LIBS})
