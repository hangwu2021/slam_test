add_library( myslam SHARED
    camera.cpp
    config.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
