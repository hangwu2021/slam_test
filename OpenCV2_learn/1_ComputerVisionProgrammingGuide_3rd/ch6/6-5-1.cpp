#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


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
        std::cerr << "Load Gray Image Failed!" << std::endl;
        return -1;
    }
    
    // sobelX
    cv::Mat sobelX;
    cv::Sobel(image, sobelX, CV_8U, 1, 0, 3, 0.4, 128);
    
    cv::imshow("sobelX", sobelX);
    
    // sobelY
    cv::Mat sobelY;
    cv::Sobel(image, sobelY, CV_8U, 0, 1, 3, 0.4, 128);
    
    cv::imshow("sobelY", sobelY);
    
    // L1 
    cv::Mat sobel;
    sobel = cv::abs(sobelX) + cv::abs(sobelY);
    
    imshow("sobel", sobel);
    
    // MaxValue
    double sobmin, sobmax;
    cv::minMaxLoc(sobel, &sobmin, &sobmax);
    
    cv::Mat sobelImage;
    sobel.convertTo(sobelImage, CV_8U, -255./sobmax, 255);
    
    cv::imshow("sobelImage", sobelImage);
    
    // Threshold
    cv::Mat sobelThresholded;
    int threshold = 80;
    cv::threshold(sobelImage, sobelThresholded, threshold, 255, cv::THRESH_BINARY);
    
    cv::imshow("sobelThresholded", sobelThresholded);
    
    // L2 & Angle
    cv::Mat sobelX2, sobelY2;
    cv::Sobel(image, sobelX2, CV_32F, 1, 0);
    cv::Sobel(image, sobelY2, CV_32F, 0, 1);
    
    cv::Mat norm, dir;
    cv::cartToPolar(sobelX2, sobelY2, norm, dir);
    
    cv::imshow("norm", norm);
    
    std::cout << dir.size() << std::endl;
    
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    
    return 0;
}
