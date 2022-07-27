#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void addSlatNoise(const cv::Mat& image, cv::Mat& noiseImage, const unsigned short N);

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image Failed!" << std::endl;
        return -1;
    }
    
    // GaussianBlur First
    cv::GaussianBlur(image, image, cv::Size(5, 5), 1.5);
    
    // Generate Noise Image
    cv::Mat noiseImage = image.clone();
    addSlatNoise(image, noiseImage, 6000);
    
    cv::imshow("noiseImage", noiseImage);
    
    
    // Median Bluring
    cv::Mat result;
    cv::medianBlur(noiseImage, result, 5);  // 5: filter size
    
    cv::imshow("medianBlurResult", result);
    
    cv::Mat meanBlurResult;
    cv::blur(noiseImage, meanBlurResult, cv::Size(3, 3));
    
    cv::imshow("meanBlurResult", meanBlurResult);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}


void addSlatNoise(const cv::Mat& image, cv::Mat& noiseImage, const unsigned short N)
{
    srand((unsigned)time(NULL));
    
    for (int i = 0; i < N; i++)
    {
        int u = rand() % image.cols;
        int v = rand() % image.rows;
        noiseImage.at<uchar>(v, u) = 255;
    }
}
