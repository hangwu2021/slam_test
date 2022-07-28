#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    
    // Gaussian Blur
    cv::GaussianBlur(image, image, cv::Size(5, 5), 1.5);
    
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT, 2, 50, 200, 100, 25, 100);
    
    // show circles
    std::vector<cv::Vec3f>::const_iterator itc = circles.begin();
    
    while (itc != circles.end())
    {
        cv::circle(image, cv::Point((*itc)[0], (*itc)[1]), (*itc)[2], cv::Scalar(255), 2);
        
        ++itc;
    }
    
    cv::imshow("image", image);
    cv::waitKey(0);
    
    return 0;
}
