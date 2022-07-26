cmake_minimum_required(VERSION 3.0)

project(ch6)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ch6 gaussNewton.cpp)

target_link_libraries(ch6 ${OpenCV_LIBS})

install(TARGETS ch6 RUNTIME DESTINATION bin)
