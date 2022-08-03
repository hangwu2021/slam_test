#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


int main(int argc, char *argv[])
{
    const std::string img1_path = "../images/1.png", img2_path = "../images/2.png";
    
    cv::Mat image1 = cv::imread(img1_path), image2 = cv::imread(img2_path);
    assert(image1.data != nullptr && image2.data != nullptr);
    
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    
    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::SIFT::create(500);
    ptrFeature2D->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    ptrFeature2D->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    
    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    
    // KeyPoint -> Point2f
    std::vector<cv::Point2f> points1, points2;
    for (auto &m : matches)
    {
        points1.push_back(keypoints1[m.queryIdx].pt);
        points2.push_back(keypoints2[m.trainIdx].pt);
    }
    
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    
    // Fundamental Matrix
    cv::Mat fundamental;
    fundamental= cv::findFundamentalMat(points1, points2);
    
    // Homography
    cv::Mat h1, h2;
    cv::stereoRectifyUncalibrated(points1, points2, fundamental, image1.size(), h1, h2);
    
    cv::Mat rectified1;
    cv::warpPerspective(image1, rectified1, h1, image1.size());
    
    cv::Mat rectified2;
    cv::warpPerspective(image2, rectified2, h2, image1.size());
    
    // Compute Difference of Visual
    cv::Mat disparity;
    cv::Ptr<cv::StereoMatcher> pStereo = cv::StereoSGBM::create(
        0, 
        128,
        5
    );
    
    pStereo->compute(rectified1, rectified2, disparity);
    
    cv::imshow("rectified1", rectified1);
    cv::imshow("rectified2", rectified2);
    
    cv::imshow("disparity", disparity);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
