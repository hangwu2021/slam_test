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
        std::cerr << "Load Image Failed!" << std::endl;
        return -1;
    }
    
    cv::Mat contuors;
    cv::Canny(image, contuors, 25, 125);    // 25: Low Threshold, 125: High Threshold
    
    cv::imshow("Canny", contuors);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
