#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


cv::Mat function()
{
    cv::Mat ima(500, 500, CV_8U, 50);
    
    return ima;
}


int main(int argc, char *argv[])
{
    // image1
    cv::Mat image1(240, 320, CV_8U, 100);
    
    cv::imshow("Image", image1);
    cv::waitKey(0);
    
    // image2
    image1.create(200, 200, CV_8U);
    image1 = 200;
    
    cv::imshow("Image", image1);
    cv::waitKey(0);
    
    cv::Mat image2(240, 320, CV_8UC3, cv::Scalar(0, 0, 255));
    //image2 = cv::Scalar(0, 0, 255);
    
    cv::imshow("Image", image2);
    cv::waitKey(0);
    
    if (argc != 2)
    {
        std::cerr << "usage: ./ch1 image.png" << std::endl;
        return -1;
    }
    
    cv::Mat image3 = cv::imread(argv[1]);
    
    cv::Mat image4(image3);
    image1 = image3;
    
    image3.copyTo(image2);
    cv::Mat image5 = image3.clone();
    
    cv::flip(image3, image3, 1);
    
    cv::imshow("Image 3", image3);
    cv::imshow("Image 1", image1);
    cv::imshow("Image 2", image2);
    cv::imshow("Image 4", image4);
    cv::imshow("Image 5", image5);
    
    cv::waitKey(0);
    
    cv::Mat gray = function();
    
    cv::imshow("Image", gray);
    cv::waitKey(0);
    
    image1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    image1.convertTo(image2, CV_32F, 1/255.0, 0.0);
    
    cv::imshow("Image", image2);
    cv::waitKey(0);
    
    return 0;
}
