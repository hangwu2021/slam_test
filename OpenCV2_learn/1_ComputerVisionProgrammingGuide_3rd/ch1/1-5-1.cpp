#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: ./ch1 image.png log.png" << std::endl;
        return -1;
    }
    
    cv::Mat image = cv::imread(argv[1], 0);
    cv::Mat logo = cv::imread(argv[2], 0);
    assert(image.data != nullptr && logo.data != nullptr);
    
    cv::resize(logo, logo, cv::Size(logo.cols/2, logo.rows/2));
    
    // Mask
    cv::Mat mask(logo);
    int i, j;
    for (i = 0; i < mask.rows; ++i)
    {
        for (j = 0; j < mask.cols; ++j)
        {
            if ( mask.at<uchar>(i, j) > 250 )
            {
                mask.at<uchar>(i, j) = 0;
            }
        }
    }
    
    cv::Mat imageROI(image, cv::Rect(image.cols - logo.cols, image.rows - logo.rows, logo.cols, logo.rows));
    logo.copyTo(imageROI, mask);
    
    cv::namedWindow("LogoImage", CV_WINDOW_AUTOSIZE);
    cv::imshow("LogoImage", image);
    
    cv::waitKey(0);
    
    return 0;
}
