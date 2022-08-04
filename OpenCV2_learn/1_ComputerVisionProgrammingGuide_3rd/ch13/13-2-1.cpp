
#include "FeatureTracker.h"

void canny(cv::Mat &img, cv::Mat &out);

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./main ./videos/video.avi" << std::endl;
        return -1;
    }
    /*
    cv::VideoCapture capture(argv[1]);
    if (! capture.isOpened())
    {
        return -1;
    }
    */
    FeatureTracker ft;
    
    cv::Mat frame;
    cv::Mat output;
    
    cv::Ptr<cv::DualTVL1OpticalFlow> tvl1 = cv::createOptFlow_DualTVL1();
    cv::Mat oflow;
    
    cv::Mat frame1 = cv::imread("../../images/1.png", 0);
    cv::Mat frame2 = cv::imread("../../images/2.png", 0);
    assert(frame1.data != nullptr && frame2.data != nullptr);
    
    tvl1->calc(frame1, frame2, oflow);
    
    std::cout << oflow.size() << std::endl;
    
    cv::Mat flowImage;
    ft.drawOpticalFlow(oflow, flowImage, 8, 2, cv::Scalar(0, 0, 0));
    
    cv::namedWindow("Extracted Foreground");
    /*bool stop = false;
    
    while(! stop)
    {
        if (! capture.read(frame))
        {
            break;
        }
        
        ft.process(frame, output);
        
        cv::imshow("Extracted Foreground", output);
        if (cv::waitKey(100) >= 0)
        {
            stop = true;
        }
    }
    
    */
    cv::imshow("Extracted Foreground", flowImage);
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    
    return 0;
}


// define
void canny(cv::Mat& img, cv::Mat& out)
{
    if (img.channels() == 3)
    {
        cv::cvtColor(img, out, cv::COLOR_BGR2GRAY);
    }
    
    cv::Canny(out, out, 100, 200);
    
    cv::threshold(out, out, 128, 255, cv::THRESH_BINARY_INV);
}
