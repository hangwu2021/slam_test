#include <iostream>
#include <random>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


void salt(cv::Mat image, int n);


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./ch2 image.png" << std::endl;
        return -1;
    }
    
    cv::Mat image = cv::imread(argv[1], 0);
    assert(image.data != nullptr);
    
    salt(image, 3000);
    
    cv::namedWindow("Image");
    cv::imshow("Image", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}


void salt(cv::Mat image, int n)
{
    std::default_random_engine generator;
    std::uniform_int_distribution<int> randomRow(0, image.rows - 1);
    std::uniform_int_distribution<int> randomCol(0, image.cols - 1);
    
    int i, j;
    for (int k = 0; k < n; ++k)
    {
        i = randomCol(generator);
        j = randomRow(generator);
        if (image.type() == CV_8UC1)
        {
            image.at<uchar>(j, i) = 255;
        }
        else if (image.type() == CV_8UC3)
        {
            image.at<cv::Vec3b>(j, i)[0] = 255;
            image.at<cv::Vec3b>(j, i)[1] = 255;
            image.at<cv::Vec3b>(j, i)[2] = 255;
        }
    }
}


