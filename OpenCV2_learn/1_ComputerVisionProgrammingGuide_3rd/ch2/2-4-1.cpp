#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void colorReduce(cv::Mat image, int div=64);

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./ch2 colorImage.png" << std::endl;
        return -1;
    }
    
    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    assert(image.data != nullptr);
    
    colorReduce(image);
    
    cv::namedWindow("ColorReduceImage", CV_WINDOW_AUTOSIZE);
    cv::imshow("ColorReduceImage", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}

void colorReduce(cv::Mat image, int div)
{
    int n = static_cast<int>(
        std::log(static_cast<double>(div)) / std::log(2.0) + 0.5
    );
    
    uchar mask = 0xFF << n;
    uchar div2 = div >> 1;
    cv::Mat_<cv::Vec3b>::iterator it = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itend = image.end<cv::Vec3b>();
    //cv::Mat_<cv::Vec3b>::const_iterator
    //cv::MatConstIterator
    for (; it != itend; ++it)
    {
        (*it)[0] &= mask;
        (*it)[0] += div2;
        
        (*it)[1] &= mask;
        (*it)[1] += div2;
        
        (*it)[2] &= mask;
        (*it)[2] += div2;
    }
}

