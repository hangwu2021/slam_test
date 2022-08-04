#ifndef FRAMEPROCESSOR_H_
#define FRAMEPROCESSOR_H_

#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>

class FrameProcessor 
{
public:
    virtual void process(cv::Mat &frame, cv::Mat &output) = 0;
};

#endif  // FRAMEPROCESSOR_H_
