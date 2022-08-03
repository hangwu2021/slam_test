#ifndef FRAMEPROCESSOR_H_
#define FRAMEPROCESSOR_H_

#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>


class FrameProcessor
{
public:
    virtual void processor(cv::Mat &input, cv::Mat &output) = 0;
};

#endif // FRAMEPROCESSOR_H_
