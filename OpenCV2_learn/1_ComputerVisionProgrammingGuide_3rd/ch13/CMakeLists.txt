cmake_minimum_required(VERSION 3.0)

project(ch13)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)

add_executable(main 13-2-1.cpp)

target_link_libraries(main myvideo)
