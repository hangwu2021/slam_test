#include "BGFGSegmentor.h"

void BGFGSegmentor::process(cv::Mat & frame, cv::Mat & output)
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

void BGFGSegmentor::setThreshold(int threshValue)
{
    this->threshold = threshValue;
}

void BGFGSegmentor::setLearningRate(double lr)
{
    this->learningRate = lr;
}

