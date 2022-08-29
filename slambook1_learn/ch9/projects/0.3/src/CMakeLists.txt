add_library(myslam SHARED 
    config.cpp
    camera.cpp
    g2o_types.cpp
)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})
