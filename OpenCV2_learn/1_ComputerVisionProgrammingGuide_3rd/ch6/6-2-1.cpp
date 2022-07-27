#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameter Error!" << std::endl;
        return -1;
    }
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Failed!" << std::endl;
        return -1;
    }
    
    cv::imshow("image", image);
    
    // Blur Result
    cv::Mat result;
    
    cv::blur(image, result, cv::Size(5, 5));
    
    cv::imshow("result", result);
    
    // Gaussian Result
    cv::Mat GaussianResult;
    
    cv::GaussianBlur(image, GaussianResult, cv::Size(5, 5), 1.5);
    
    cv::imshow("GaussianBlur", GaussianResult);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    double sigma = 1.5;
    cv::Mat gauss = cv::getGaussianKernel(9, sigma, CV_32F);
    
    std::cout << gauss << std::endl;
    
    return 0;
}
