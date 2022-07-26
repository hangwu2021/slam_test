cmake_minimum_required(VERSION 3.0)

project(ch5)

# Sophus
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(rgbdVision joinMap.cpp)

target_link_libraries(rgbdVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt)
