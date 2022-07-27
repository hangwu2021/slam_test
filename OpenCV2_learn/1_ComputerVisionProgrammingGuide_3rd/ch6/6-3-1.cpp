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
    
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image Failed!" << std::endl;
        return -1;
    }
    
    // Reduce the High Frequency First
    cv::GaussianBlur(image, image, cv::Size(11, 11), 2.0);
    
    cv::imshow("image", image);
    
    // Reduced Result Image
    cv::Mat reduced(image.rows/4, image.cols/4, CV_8U);
    for (int i = 0; i < reduced.rows; i++)
    {
        for (int j = 0; j < reduced.cols; j++)
        {
            reduced.at<uchar>(i, j) = image.at<uchar>(i*4, j*4);
        }
    }
    
    cv::imshow("reduced", reduced);
    
    // PyDown Image
    cv::Mat pyReduced;
    cv::pyrDown(image, pyReduced);
    
    cv::imshow("pyReduced", pyReduced);
    
    cv::Mat newImage;
    cv::resize(pyReduced, newImage, cv::Size(), 2, 2, cv::INTER_NEAREST);
    cv::imshow("newImage", newImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
