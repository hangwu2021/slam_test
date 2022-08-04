#ifndef FEATURETRACKER_H_
#define FEATURETRACKER_H_

#include "FrameProcessor.h"

class FeatureTracker : public FrameProcessor
{
public:
    FeatureTracker() : max_count(500), qlevel(0.01), minDist(10.) {}
    
    void process(cv::Mat & frame, cv::Mat & output) override;
    bool addNewPoints();
    void detectFeaturePoints();
    bool acceptTrackedPoint(int i);
    void handleTrackedPoints(cv::Mat &frame, cv::Mat &output);
    
public:
    void drawOpticalFlow(const cv::Mat &flow, cv::Mat &flowImage, int stride, float scale, const cv::Scalar &color);
    
private:
    cv::Mat gray;
    cv::Mat gray_prev;
    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> initial;
    std::vector<cv::Point2f> features;
    int max_count;
    double qlevel;
    double minDist;
    std::vector<uchar> status;
    std::vector<float> err;
};

#endif  // FEATURETRACKER_H_
