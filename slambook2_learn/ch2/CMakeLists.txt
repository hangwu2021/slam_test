cmake_minimum_required( VERSION 2.8 )

project( UseHello )

set( CMAKE_BUILD_TYPE "Debug")

add_executable( useHello useHello.cpp )

add_library( hello_shared SHARED libHelloSLAM.cpp )

target_link_libraries( useHello hello_shared )
