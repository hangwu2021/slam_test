add_library(myslam SHARED
	config.cpp
	camera.cpp
	frame.cpp
	map.cpp
	mappoint.cpp
	visual_odometry.cpp
)

target_link_libraries(myslam ${THIRD_PARTH_LIBS} fmt::fmt)
