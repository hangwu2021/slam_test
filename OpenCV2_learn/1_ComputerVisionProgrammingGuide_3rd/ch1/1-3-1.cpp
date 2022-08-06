#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Event Callback
void onMouse(int event, int x, int y, int flags, void* param);


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    cv::Mat image;
    
    image = cv::imread(argv[1], 0);
    if (image.empty())
    {
        std::cerr << "Load Image Failed!" << std::endl;
        return -1;
    }
    
    std::cout << "This image is " << image.rows << " x " << image.cols << " x " << image.channels() << std::endl;
    
    cv::circle(image, cv::Point(360, 300), 65, 0, 3);
    
    cv::putText(image, "This is a buffalo.", cv::Point(268, 405), cv::FONT_HERSHEY_PLAIN, 2.0, 255, 2);
    
    cv::namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("Original Image", image);
    
    cv::setMouseCallback("Original Image", onMouse, reinterpret_cast<void*>(&image));
    
    // flip
    cv::Mat result;
    cv::flip(image, result, 1);
    
    cv::namedWindow("Output Image");
    cv::imshow("Output Image", result);
    
    // save result
    cv::imwrite("../output.bmp", result);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}

void onMouse(int event, int x, int y, int flags, void* param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat *>(param);
    
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            std::cout << "at (" << x << ", " << y << ") value is: " << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
            break;
    }
}
