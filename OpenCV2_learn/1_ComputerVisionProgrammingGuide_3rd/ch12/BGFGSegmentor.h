#ifndef BGFGSEGMENTOR_H_
#define BGFGSEGMENTOR_H_

#include "FrameProcessor.h"

class BGFGSegmentor : public FrameProcessor
{
public:
    void processor(cv::Mat &frame, cv::Mat &output)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        if (background.empty())
        {
            gray.convertTo(background, CV_32F);
        }
        
        background.convertTo(backImage, CV_8U);
        
        cv::absdiff(backImage, gray, foreground);
        cv::threshold(foreground, output, threshold, 255, cv::THRESH_BINARY_INV);
        cv::accumulateWeighted(gray, background, learningRate, output);
    }
    
    void setThreshold(int thvalue)
    {
        threshold = thvalue;
    }
    
private:
    cv::Mat gray;
    cv::Mat background;
    cv::Mat backImage;
    cv::Mat foreground;
    
    double  learningRate;
    int     threshold;
};

#endif BGFGSEGMENTOR_H_
