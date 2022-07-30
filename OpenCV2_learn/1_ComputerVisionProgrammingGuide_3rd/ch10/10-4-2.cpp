#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching.hpp>


int main(int argc, char *argv[])
{
    std::vector<cv::Mat> images;
    
    images.push_back(cv::imread(argv[1]));
    images.push_back(cv::imread(argv[2]));
    
    cv::Mat panorama;
    cv::Stitcher stitcher = cv::Stitcher::createDefault();
    cv::Stitcher::Status status = stitcher.stitch(images, panorama);
    
    cv::imshow("Panorama", panorama);
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
