cmake_minimum_required(VERSION 3.0)

project(ch1)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ch1 1-5-1.cpp)

target_link_libraries(ch1 ${OpenCV_LIBS})
