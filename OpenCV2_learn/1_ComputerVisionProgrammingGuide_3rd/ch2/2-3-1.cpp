#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void colorReduce(cv::Mat image, int div=64);
void colorReduce2(cv::Mat image, int div=64);

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: ./ch2 image.png 1/0" << std::endl;
        
        return -1;
    }
    
    cv::Mat image;
    image = cv::imread(argv[1], std::atoi(argv[2]));
    assert(image.data != nullptr);
    
    colorReduce(image, 64);
    
    cv::namedWindow("Image");
    cv::imshow("Image", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}

void colorReduce(cv::Mat image, int div)
{
    int n1 = image.rows;
    int nc = image.cols * image.channels();
    
    for (int j = 0; j < n1; ++j)
    {
        uchar *data = image.ptr<uchar>(j);
        for (int i = 0; i < nc; ++i)
        {
            data[i] = data[i]/div*div + div/2;
        }
    }
}

void colorReduce2(cv::Mat image, int div)
{
    int n1 = image.rows;
    int nc = image.cols * image.channels();
    
    if (image.isContinuous())
    {
        nc = nc * n1;
        n1 = 1;
    }
    
    int n = static_cast<int>(
        log(static_cast<double>(div))/log(2.0) + 0.5
    );
    
    uchar mask = 0xFF << n;
    uchar div2 = div >> 1;
    
    for (int j = 0; j < n1; ++j)
    {
        uchar *data = image.ptr<uchar>(j);
        for (int i = 0; i < nc; ++i)
        {
            *data &= mask;
            *data++ += div2;
        }
    }
}
