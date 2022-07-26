cmake_minimum_required(VERSION 3.0)

project(ch5)

# Eigen
include_directories( "/usr/include/eigen3" )

# OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)

target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
