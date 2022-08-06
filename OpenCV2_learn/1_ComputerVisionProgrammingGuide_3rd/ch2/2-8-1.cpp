#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void wave(const cv::Mat &image, cv::Mat &result);

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./ch2 image.png" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    assert(image.data != nullptr);
    
    cv::Mat result;
    wave(image, result);
    
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::imshow("result", result);    
    
    cv::waitKey(0);
    
    cv::destroyAllWindows();
    
    return 0;
}

void wave(const cv::Mat& image, cv::Mat& result)
{
    cv::Mat srcX(image.rows, image.cols, CV_32F);
    cv::Mat srcY(image.rows, image.cols, CV_32F);
    
    for (int i = 0; i < image.rows; ++i)
    {
        for (int j = 0; j < image.cols; ++j)
        {
            srcX.at<float>(i, j) = j;
            srcY.at<float>(i, j) = i + 5 * std::sin(j/10.0);
        }
    }
    
    cv::remap(image, result, srcX, srcY, cv::INTER_LINEAR);
}

