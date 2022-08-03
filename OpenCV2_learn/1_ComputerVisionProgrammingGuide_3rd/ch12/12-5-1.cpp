#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/bgsegm.hpp>

int main(int argc, char *argv[])
{
    cv::VideoCapture capture(argv[1]);
    if (!capture.isOpened())
    {
        return 0;
    }
    
    cv::Mat frame;
    cv::Mat foreground;
    cv::Mat background;
    
    cv::namedWindow("Extracted Foreground");
    
    cv::Ptr<cv::BackgroundSubtractor> ptrMOG = cv::bgsegm::createBackgroundSubtractorMOG();
    
    bool stop = false;
    while (!stop)
    {
        if (!capture.read(frame))
        {
            break;
        }
        
        ptrMOG->apply(frame, foreground, 0.01);
        
        cv::threshold(foreground, foreground, 128, 255, cv::THRESH_BINARY_INV);
        
        cv::imshow("Extracted Foreground", foreground);
        
        if (cv::waitKey(100) >= 0)
        {
            stop = true;
        }
    }
    
    
    return 0;
}
