#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    cv::VideoCapture capture(argv[1]);
    if (!capture.isOpened())
    {
        std::cerr << "Open Video Failed!" << std::endl;
        return -1;
    }
    
    double rate = capture.get(CV_CAP_PROP_FPS);
    long total_frames = static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));
    
    std::cout << "total_frames = " << total_frames << std::endl;
    
    bool stop(false);
    cv::Mat frame;
    
    int delay = 1000 / rate;
    
    cv::namedWindow("Extracted Frame");
    
    while (! stop)
    {
        if (! capture.read(frame))  // if not empty, continue.
        {
            break;
        }
        
        cv::imshow("Extracted Frame", frame);
        
        if (cv::waitKey(delay) >= 0)
        {
            stop = true;
        }
    }
    
    capture.release();
    
    return 0;
}
