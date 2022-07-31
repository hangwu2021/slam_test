#include <iostream>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

const std::string file_1 = "./LK1.png";
const std::string file_2 = "./LK2.png";


class OpticalFlowTracker
{
public:
    OpticalFlowTracker(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const std::vector<cv::KeyPoint> &kp1_,
        std::vector<cv::KeyPoint> &kp2_,
        std::vector<bool> &success_,
        bool inverse_ = true,
        bool has_initial_ = false
    ) : img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_), has_initial(has_initial_)
    {
        
    }
    
    void calculateOpticalFlow(const cv::Range &range);
    
private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const std::vector<cv::KeyPoint> &kp1;
    std::vector<cv::KeyPoint> &kp2;
    std::vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

void OpticalFlowTracker::calculateOpticalFlow(const cv::Range& range)
{
    
}


void OpticalFlowSingleLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse = false,
    bool has_initial = false
);

void OpticalFlowMultiLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse = false
);

inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    return 0.0;
}



int main(int argc, char *argv[])
{
    cv::Mat img1 = cv::imread(file_1, 0);
    cv::Mat img2 = cv::imread(file_2, 0);
    
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);
    
    
    
    
    return 0;
}


void OpticalFlowSingleLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse,
    bool has_initial
)
{
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    cv::parallel_for_(cv::Range(0, kp1.size()), std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, std::placeholders::_1));
}

