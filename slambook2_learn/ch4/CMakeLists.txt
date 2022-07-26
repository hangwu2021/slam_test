cmake_minimum_required(VERSION 3.0)

project(ch4)

# Eigen
include_directories("/usr/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(ch4 trajectoryError.cpp)

target_link_libraries(ch4 ${Pangolin_LIBRARIES} fmt)

install(TARGETS ch4 RUNTIME DESTINATION bin)
