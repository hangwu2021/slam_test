# include "VisualTracker.h"

void VisualTracker::setBoundingBox(const cv::Rect2d &bb)
{
    box = bb;
    reset = true;
}

void VisualTracker::process(cv::Mat & frame, cv::Mat & output)
{
    if(reset)
    {
        reset = false;
        tracker->init(frame, box);
    }
    else
    {
        tracker->update(frame, box);
    }
    
    frame.copyTo(output);
    
    cv::rectangle(output, box, cv::Scalar(255, 255, 255), 2);
}
