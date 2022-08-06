/*选取8个以角点为特征的特征检测器进行测试，包括：

HARRIS
GFTT (Shi-tomasi)
SIFT
SURF
FAST
STAR
ORB (oriented BRIEF)
BRISK
*/

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./ch8 image.png" << std::endl;
        
        return -1;
    }
    
    // Load Image
    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    assert(image.data != nullptr);
    
    // Declare Detctor
    std::vector<std::string> detectorsName = {"HARRIS", "GFTT", "SIFT", "SURF", "FAST", "STAR", "ORB", "BRISK"};
    
    std::cout << "Detector Name" << "\t" << "Number of corners" << "\t" << "Time used" << "\t\t" << "Efficiency Report" << "\n" << std::endl;
    
    cv::Ptr<cv::FeatureDetector> detector;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat result;
    for (std::string &detectorName : detectorsName)
    {
        keypoints.clear();

        double t = (double)cv::getTickCount();
        
        if (std::strcmp(detectorName.c_str(), "HARRIS"))
        {
            detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "GFTT"))
        {
            detector = cv::GFTTDetector::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "SIFT"))
        {
            detector = cv::SIFT::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "SURF"))
        {
            detector = cv::xfeatures2d::SURF::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "FAST"))
        {
            detector = cv::FastFeatureDetector::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "STAR"))
        {
            detector = cv::xfeatures2d::StarDetector::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "ORB"))
        {
            detector = cv::ORB::create();
        }
        
        if (std::strcmp(detectorName.c_str(), "BRISK"))
        {
            detector = cv::BRISK::create();
        }
        
        detector->detect(image, keypoints);
        
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        double efficiency = keypoints.size() / t/ 1000;
        std::cout << detectorName << "\t\t" << keypoints.size() << "\t\t\t" << t << "\t\t" << efficiency << "\n" << std::endl;;
        
        cv::drawKeypoints(image, keypoints, result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        
        cv::imshow(detectorName, result);
    }
    
    cv::waitKey(0);
    
    cv::destroyAllWindows();
    
    return 0;
}
