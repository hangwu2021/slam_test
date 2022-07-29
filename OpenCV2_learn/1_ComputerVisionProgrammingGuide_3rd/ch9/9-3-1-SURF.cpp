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

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    
    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);
    
    // Detect keypoints
    ptrFeature2D->detect(image1, keypoints1);
    ptrFeature2D->detect(image2, keypoints2);
    
    // Compute descriptors
    cv::Mat descriptors1, descriptors2;
    ptrFeature2D->compute(image1, keypoints1, descriptors1);
    ptrFeature2D->compute(image2, keypoints2, descriptors2);
    
    // Construct Matcher
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    
    // Show Result
    cv::Mat matchImage;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));
    
    cv::imshow("matchImage-SURF", matchImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
