cmake_minimum_required(VERSION 3.0)

project(ch2)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ch2 2-8-1.cpp)

target_link_libraries(ch2 ${OpenCV_LIBS})
