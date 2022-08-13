#ifndef COLORDETECTOR_H_
#define COLORDETECTOR_H_

#include "CommonInclude.h"

class ColorDetector 
{
public:
    ColorDetector();
    ColorDetector(uchar blue, uchar green, uchar red, int maxDist);
    
    void setTargetColor(uchar blue, uchar green, uchar red);
    void setTargetColor(cv::Vec3b color);
    cv::Vec3b getTargetColor() const;
    int getDistanceToTargetColor(const cv::Vec3b &color) const;
    int getColorDistance(const cv::Vec3b &color1, const cv::Vec3b &color2) const;
    cv::Mat process(const cv::Mat &image);
    cv::Mat process_opencv(const cv::Mat &image);
    void setColorDistanceThreshold(int distance);
    int getColorDistanceThreshold() const;
    
private:
    int maxDist;
    cv::Vec3b target;
    cv::Mat result;
};

#endif  // COLORDETECTOR_H_
