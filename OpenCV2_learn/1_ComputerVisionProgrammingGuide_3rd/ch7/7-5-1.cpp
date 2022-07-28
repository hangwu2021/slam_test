#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image = cv::imread(argv[1]);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image" << std::endl;
        return -1;
    }
    
    cv::Mat markers;
    cv::watershed(image, markers);
    //markers.convertTo(markers, CV_32S);
    
    //cv::imshow("markers", markers);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
