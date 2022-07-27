cmake_minimum_required(VERSION 3.0)

project(1_ComputerVisionProgrammingGuide_3rd)

# OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main ch6/6-6-1.cpp)

target_link_libraries(main ${OpenCV_LIBS})
