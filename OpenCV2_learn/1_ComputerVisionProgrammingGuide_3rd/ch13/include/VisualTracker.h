#ifndef VISUALTRACKER_H_
#define VISUALTRACKER_H_

#include "FrameProcessor.h"

class VisualTracker : public FrameProcessor
{
public:
    VisualTracker(cv::Ptr<cv::Tracker> tracker) : reset(true), tracker(tracker) {}
    
    void process(cv::Mat & frame, cv::Mat & output) override;
    
    void setBoundingBox(const cv::Rect2d &bb);
    
private:
    cv::Rect2d box;
    bool reset;
    cv::Ptr<cv::Tracker> tracker;
};

#endif  // VISUALTRACKER_H_
