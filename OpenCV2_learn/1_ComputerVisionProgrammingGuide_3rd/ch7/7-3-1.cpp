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
    
    // Canny
    cv::Mat contours;
    cv::Canny(image, contours, 125, 350);
    
    // Hough Transform to Detect Lines
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, CV_PI/180, 60);
    
    // show lines
    std::vector<cv::Vec2f>::const_iterator it = lines.begin();
    while (it != lines.end())
    {
        float rho = (*it)[0];
        float theta = (*it)[1];
        
        if (theta < CV_PI/4.0 || theta > 3.0 * CV_PI/4.0)
        {
            cv::Point pt1(rho/std::cos(theta), 0);
            cv::Point pt2((rho - contours.rows*std::sin(theta)) / std::cos(theta), contours.rows);
            
            cv::line(image, pt1, pt2, cv::Scalar(255), 1);
        }
        else
        {
            cv::Point pt1(0, rho/std::sin(theta));
            cv::Point pt2(contours.cols, (rho-contours.cols*std::cos(theta))/std::sin(theta));
            
            cv::line(image, pt1, pt2, cv::Scalar(255), 1);
        }
        
        ++it;
    }
    
    cv::imshow("lines", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
