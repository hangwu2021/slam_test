#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class RobustMatcher
{
public:
    RobustMatcher(const cv::Ptr<cv::FeatureDetector> &detector, 
                  const cv::Ptr<cv::DescriptorExtractor> &descriptor=cv::Ptr<cv::DescriptorExtractor>())
    : detector(detector), descriptor(descriptor), normType(cv::NORM_L2), ratio(0.8f), refineF(true), refineM(true), confidence(0.98), distance(1.0)
    {
        if ( !(this->descriptor) )
        {
            this->descriptor = this->detector;
        }
    }
    
    // RANSAC
    cv::Mat match(cv::Mat &image1, cv::Mat &image2, std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2)
    {
        // 1. Detect Key Points
        detector->detect(image1, keypoints1);
        detector->detect(image2, keypoints2);
        
        // 2. Extract Feature Descriptors
        cv::Mat descriptors1, descriptors2;
        descriptor->compute(image1, keypoints1, descriptors1);
        descriptor->compute(image2, keypoints2, descriptors2);
        
        // 3. Match Descriptors of Two Images
        cv::BFMatcher matcher(normType, true);
        std::vector<cv::DMatch> outputMatches;
        matcher.match(descriptors1, descriptors2, outputMatches);
        
        // 4. RANSAC Validate Matched
        cv::Mat fundamental = ransacTest(outputMatches, keypoints1, keypoints2, matches);
        
        return fundamental;
    }
    
    // RANSAC Test
    cv::Mat ransacTest(
        const std::vector<cv::DMatch> &matches,
        std::vector<cv::KeyPoint> &keypoints1,
        std::vector<cv::KeyPoint> &keypoints2,
        std::vector<cv::DMatch> &outMatches
    )
    {
        std::vector<cv::Point2f> points1, points2;
        for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
        {
            points1.push_back(keypoints1[it->queryIdx].pt);
            points2.push_back(keypoints2[it->trainIdx].pt);
        }
        
        std::vector<uchar> inliers(points1.size(), 0);
        cv::Mat fundamental = cv::findFundamentalMat(points1, points2, inliers, cv::FM_RANSAC, distance, confidence);
        
        std::vector<uchar>::const_iterator itIn = inliers.begin();
        std::vector<cv::DMatch>::const_iterator itM = matches.begin();
        for (; itIn != inliers.end(); ++itIn, ++itM)
        {
            if ( *itIn ) 
            {
                outMatches.push_back( *itM );
            }
        }
        
        return fundamental;
    }
    
private:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    int normType;
    float ratio;
    bool refineF;
    bool refineM;
    double distance;
    double confidence;
};


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    cv::Mat image1 = cv::imread(argv[1], 0), image2 = cv::imread(argv[2], 0);
    assert(image1.data != nullptr && image2.data != nullptr);
    
    RobustMatcher rmatcher(cv::SIFT::create(250));
    
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    
    cv::Mat fundamental = rmatcher.match(image1, image2, matches, keypoints1, keypoints2);
    
    std::cout << fundamental << std::endl;
    
    
    return 0;
}
