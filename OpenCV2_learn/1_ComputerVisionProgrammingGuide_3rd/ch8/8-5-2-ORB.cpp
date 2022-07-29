#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image" << std::endl;
        return -1;
    }
    
    // Source Image is too large to not see beter.
    cv::resize(image, image, cv::Size(image.rows/3, image.cols/3));
    
    //ORB: Keypoints Detecting
    cv::Ptr<cv::ORB> ptrORB = cv::ORB::create(75, 1.2, 16);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat featureImage;
    
    ptrORB->detect(image, keypoints);
    
    cv::drawKeypoints(image, keypoints, featureImage, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    cv::imshow("ORB", featureImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}




