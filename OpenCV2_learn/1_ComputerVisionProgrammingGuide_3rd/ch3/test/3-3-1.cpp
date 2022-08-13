#include "CommonInclude.h"

int main(int argc, char *argv[])
{
    cv::Rect rectangle(5, 70, 260, 120);
    
    cv::Mat image = cv::imread(argv[1]);
    
    cv::Mat result;
    cv::Mat bgModel, fgModel;
    
    cv::grabCut(image, result, rectangle, bgModel, fgModel, 5, cv::GC_INIT_WITH_RECT);
    
    cv::compare(result, cv::GC_PR_FGD, result, cv::CMP_EQ);
    
    cv::Mat foreground(image.size(), CV_8UC3, cv::Scalar(255, 255, 255));
    image.copyTo(foreground, result);
    
    cv::imshow("result", foreground);
    cv::waitKey(0);
    
    return 0;
}
