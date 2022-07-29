#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image1 = cv::imread(argv[1]), image2 = cv::imread(argv[2]);
    assert(image1.data != nullptr && image2.data != nullptr);

    // Initialize
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    
    // Construct ORB
    cv::Ptr<cv::Feature2D> feature = cv::ORB::create(60);
    
    feature->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    feature->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    
    // Construct Matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    //cv::xfeatures2d::FREAK::create()
    // Show Result
    cv::Mat matchImage;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));
    
    cv::imshow("matchImage-ORB", matchImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}

