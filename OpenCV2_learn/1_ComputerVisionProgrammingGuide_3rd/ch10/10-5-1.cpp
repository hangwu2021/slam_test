#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class TargetMatcher
{
public:
    TargetMatcher(const cv::Ptr<cv::FeatureDetector> &detector, 
                  const cv::Ptr<cv::DescriptorExtractor> &descriptor=cv::Ptr<cv::DescriptorExtractor>())
    {
        
    }
    
private:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    cv::Mat target;
    int normType;
    double distance;
    int numberOfLevels;
    double scaleFactor;
    std::vector<cv::Mat> pyramid;
    std::vector<std::vector<cv::KeyPoint>> pyrKeypoints;
    std::vector<cv::Mat> pyrDescriptors;
};


int main(int argc, char *argv[])
{
    // Waitting for me ... 
    
    
    
    return 0;
}

