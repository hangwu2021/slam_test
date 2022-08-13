cmake_minimum_required(VERSION 3.0)

project(ch3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# OpenCV 3.4.15
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# run test 
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
