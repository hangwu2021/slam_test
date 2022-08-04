
include_directories(${PROJECT_SOURCE_DIR}/../include)

add_library(myvideo SHARED 
    VideoProcessor.cpp
    BGFGSegmentor.cpp
    FeatureTracker.cpp
    VisualTracker.cpp
)

target_link_libraries(myvideo ${OpenCV_LIBS})

