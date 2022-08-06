#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


int main(int argc, char *argv[])
{
    cv::Matx33d matrix(3.0, 2.0, 1.0, 2.0, 1.0, 3.0, 1.0, 2.0, 3.0);
    
    cv::Matx31d vector(5.0, 1.0, 3.0);
    
    cv::Matx31d result = matrix * vector;
    
    std::cout << matrix << std::endl;
    std::cout << vector << std::endl;
    std::cout << result << std::endl;
    
    std::cout << matrix.inv() << std::endl;
    std::cout << matrix * matrix.inv() << std::endl;
    
    return 0;
}
