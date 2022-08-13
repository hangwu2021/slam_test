#include "ColorDetector.h"

ColorDetector::ColorDetector() :maxDist(100), target(0, 0, 0)
{
    
}

ColorDetector::ColorDetector(uchar blue, uchar green, uchar red, int maxDist)
{
    this->maxDist = maxDist;
    this->target = cv::Vec3b(blue, green, red);
}

int ColorDetector::getDistanceToTargetColor(const cv::Vec3b& color) const
{
    return getColorDistance(color, target);
}

int ColorDetector::getColorDistance(const cv::Vec3b& color1, const cv::Vec3b& color2) const
{
    return std::abs(color1[0] - color2[0]) + std::abs(color1[1] - color2[1]) + std::abs(color1[2] - color2[2]);
}

cv::Mat ColorDetector::process(const cv::Mat& image)
{
    result.create(image.size(), CV_8U);
    
    cv::Mat_<cv::Vec3b>::const_iterator it = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = image.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = result.begin<uchar>();
    
    for ( ; it != itend; ++it, ++itout)
    {
        if (getDistanceToTargetColor(*it) <= maxDist)
        {
            *itout = 255;
        }
        else 
        {
            *itout = 0;
        }
    }
    
    return result;
}

cv::Mat ColorDetector::process_opencv(const cv::Mat &image)
{
    cv::Mat output;
    
    cv::absdiff(image, cv::Scalar(target), output);
    
    std::vector<cv::Mat> images;
    cv::split(output, images);
    
    output = images[0] + images[1] + images[2];
    
    cv::threshold(output, output, maxDist, 255, cv::THRESH_BINARY_INV);
    
    return output;
}

void ColorDetector::setColorDistanceThreshold(int distance)
{
    if (distance < 0)
    {
        distance = 0;
    }
    
    maxDist = distance;
}

int ColorDetector::getColorDistanceThreshold() const
{
    return this->maxDist;
}

void ColorDetector::setTargetColor(uchar blue, uchar green, uchar red)
{
    this->target = cv::Vec3b(blue, green, red);
}

void ColorDetector::setTargetColor(cv::Vec3b color)
{
    this->target = color;
}

cv::Vec3b ColorDetector::getTargetColor() const
{
    return target;
}



