add_library(myslam SHARED 
    config.cpp
)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})
