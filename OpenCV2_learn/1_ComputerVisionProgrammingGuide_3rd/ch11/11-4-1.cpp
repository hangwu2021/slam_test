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
    
    // Essential from image1 and image2
    cv::Mat inliers;
    cv::Mat Matrix = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat essential = cv::findEssentialMat(
        points1,
        points2,
        Matrix,
        cv::RANSAC,
        0.9,
        1.0,
        inliers
    );
    
    std::cout << "Essential = \n" << essential << std::endl;
    
    // recover pose
    cv::Mat rotation, translation;
    cv::recoverPose(essential, points1, points2, Matrix, rotation, translation, inliers);
    
    std::cout << "R = \n" << rotation << "\nt = \n" << translation << std::endl;
    
    // triangulate
    cv::Mat projection2(3, 4, CV_64F);
    rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(projection2.colRange(3, 4));
    
    cv::Mat projection1(3, 4, CV_64F, 0.0);
    cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
    diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
    
    
    
    //std::cout << projection1 << std::endl;
    
    
    
    return 0;
}
