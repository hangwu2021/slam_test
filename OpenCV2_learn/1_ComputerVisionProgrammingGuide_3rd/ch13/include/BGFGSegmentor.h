#ifndef BGFGSEGMENTOR_H_
#define BGFGSEGMENTOR_H_

#include "FrameProcessor.h"

class BGFGSegmentor : public FrameProcessor
{
public:
    void process(cv::Mat & frame, cv::Mat & output);
    void setThreshold(int threshValue);
    void setLearningRate(double lr);
    
private:
    cv::Mat gray;
    cv::Mat background;
    cv::Mat backImage;
    cv::Mat foreground;
    double learningRate;
    int threshold;
};

#endif  // BGFGSEGMENTOR_H_
