add_library(color SHARED
    ColorDetector.cpp
)

target_link_libraries(color ${OpenCV_LIBS})
