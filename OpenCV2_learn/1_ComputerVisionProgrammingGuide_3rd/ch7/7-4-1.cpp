#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "" << std::endl;
        return -1;
    }
    
    // Reduce Noise
    cv::GaussianBlur(image, image, cv::Size(5, 5), 1.5);
    
    // Detect contours
    cv::Mat contours;
    cv::Canny(image, contours, 50, 200);
    
    // Detect lines
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, CV_PI/180, 60);
   
    // Lines
    int n = 0;
    cv::Mat oneline(contours.size(), CV_8U, cv::Scalar(0));
    
    cv::line(oneline, cv::Point(lines[n][0], lines[n][1]), cv::Point(lines[n][2], lines[n][3]), cv::Scalar(255), 3);
    
    cv::bitwise_and(contours, oneline, oneline);
    
    
    cv::imshow("image", oneline);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
