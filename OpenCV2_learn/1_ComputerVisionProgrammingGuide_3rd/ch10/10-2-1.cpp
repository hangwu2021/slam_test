#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    cv::Mat img_1, img_2;
    img_1 = cv::imread(argv[1], 0), img_2 = cv::imread(argv[2], 0);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    cv::Ptr<cv::SIFT> ptrFeature2D = cv::SIFT::create();
    
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    ptrFeature2D->detect(img_1, keypoints_1);
    ptrFeature2D->detect(img_2, keypoints_2);
    
    cv::Mat descriptors_1, descriptors_2;
    ptrFeature2D->compute(img_1, keypoints_1, descriptors_1);
    ptrFeature2D->compute(img_2, keypoints_2, descriptors_2);
    
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);
    
    int N = 9;  // Don't too small, nor Error!
    std::nth_element(matches.begin(), matches.begin() + N, matches.end());
    matches.erase(matches.begin() + N, matches.end());
    
    std::vector<cv::Point2f> selPoints1, selPoints2;
    for (auto &m : matches)
    {
        selPoints1.push_back(keypoints_1[m.queryIdx].pt);
        selPoints2.push_back(keypoints_2[m.trainIdx].pt);
    }
    std::cout << selPoints1[0] << std::endl;
    
    cv::Mat fundamental = cv::findFundamentalMat(selPoints1, selPoints2, CV_FM_7POINT);
    std::cout << fundamental.size() << std::endl;
    
    // Show Result
    std::vector<cv::Vec3f> lines1;
    cv::computeCorrespondEpilines(
        selPoints1,
        1,
        fundamental,
        lines1
    );
    
    for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin(); it != lines1.end(); ++it)
    {
        cv::line(img_2, cv::Point(0, -(*it)[2]/(*it)[1]), cv::Point(img_2.cols, -((*it)[2] + (*it)[0] * img_2.cols) / (*it)[1]), cv::Scalar(255, 255, 255));
    }
    
    cv::imshow("result", img_2);
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
