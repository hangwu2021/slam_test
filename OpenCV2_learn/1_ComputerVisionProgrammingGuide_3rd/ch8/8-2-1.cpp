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
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image" << std::endl;
        return -1;
    }
    
    // Source Image is too large to not see beter.
    cv::resize(image, image, cv::Size(image.rows/3, image.cols/3));
    
    cv::imshow("image", image);
    
    // Detect Harris Corner
    cv::Mat cornerStrength;
    cv::cornerHarris(
        image,
        cornerStrength,
        3,
        3,
        0.01
    );
    
    cv::Mat harrisCorners;
    double threshold = 0.0001;
    cv::threshold(
        cornerStrength, 
        harrisCorners,
        threshold,
        255,
        cv::THRESH_BINARY
    );
    
    cv::imshow("harrisCorners", harrisCorners);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
