#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: ./ch2 image1.png image2.png" << std::endl;
        return -1;
    }
    
    cv::Mat image1, image2;
    image1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    image2 = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    assert(image1.data != nullptr && image2.data != nullptr);
    
    // Resize Image1
    double rowRatio = (double)(image1.rows+0.0) / image2.rows;
    double colRatio = (double)(image1.cols+0.0) / image2.cols;
    
    cv::resize(image1, image1, cv::Size(image1.cols/colRatio, image1.rows/rowRatio));
    
    // Add images
    cv::Mat result;
    cv::addWeighted(image1, 0.4, image2, 0.7, 0.2, result);
    
    cv::namedWindow("Image1", CV_WINDOW_AUTOSIZE);
    cv::imshow("Image1", image1);
    
    cv::namedWindow("Image2", CV_WINDOW_AUTOSIZE);
    cv::imshow("Image2", image2);
    
    cv::namedWindow("AddedResult", CV_WINDOW_AUTOSIZE);
    cv::imshow("AddedResult", result);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
