#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./build/main ./images/calib.txt" << std::endl;
        return -1;
    }
    
    std::string path;
    std::ifstream fin(argv[1]);
    
    std::cout << "Start Extracting Corner Point..." << std::endl;
    
    unsigned int image_count = 0;
    cv::Size image_size;
    cv::Size board_size = cv::Size(4, 6);
    std::vector<cv::Point2f> image_points_buf;
    std::vector<std::vector<cv::Point2f>> image_points_seq;
    int count = -1;
    
    while(getline(fin, path))
    {
        image_count++;
        std::cout << "image_count = " << image_count << std::endl;
        std::cout << "count = " << count << std::endl;
        
        cv::Mat imageInput;
        imageInput = cv::imread(path);
        
        cv::resize(imageInput, imageInput, cv::Size(imageInput.rows/4, imageInput.cols/4));
        
        if (1 == image_count && imageInput.data != nullptr)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            std::cout << "image_size.width = " << image_size.width << std::endl;
            std::cout << "image_size.height = " << image_size.height << std::endl;
        }
        
        if (0 == cv::findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            std::cout << "can not find any corner point." << std::endl;
            return -1;
        }
        else
        {
            cv::Mat view_gray;
            cv::cvtColor(imageInput, view_gray, CV_BGR2GRAY);
            cv::find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5));
            image_points_seq.push_back(image_points_buf);
            
            cv::drawChessboardCorners(view_gray, board_size, image_points_buf, false);
            cv::imshow("Camera Calibration", view_gray);
            
            cv::waitKey(3000);
        }
    }
    
    return 0;
}

