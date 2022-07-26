cmake_minimum_required(VERSION 3.0)

project(ch3)

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories( "/usr/include/eigen3" )

add_executable(ch3 plotTrajectory.cpp)

target_link_libraries(ch3 ${Pangolin_LIBRARIES})

install(TARGETS ch3 RUNTIME DESTINATION bin)
