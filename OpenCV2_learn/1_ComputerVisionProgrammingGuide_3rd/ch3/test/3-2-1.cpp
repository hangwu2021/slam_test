#include "ColorDetector.h"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "usage: ./ch3 image.png" << std::endl;
        return -1;
    }
    
    ColorDetector color_detector;
    
    cv::Mat image = cv::imread(argv[1]);
    assert(image.data != nullptr);
    
    color_detector.setTargetColor(230, 190, 130);
    
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::Mat result = color_detector.process_opencv(image);
    cv::imshow("result", result);
    
    cv::waitKey(0);
    
    return 0;
}
