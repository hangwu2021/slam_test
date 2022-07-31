#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: feature_extraction img1 img2" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image1 = cv::imread(argv[1]), image2 = cv::imread(argv[2]);
    
    assert(image1.data != nullptr && image2.data != nullptr);
    
    // Initialization
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    // Step1: detect corner points
    detector->detect(image1, keypoints_1);
    detector->detect(image2, keypoints_2);
    
    // Step2: detect descriptors
    descriptor->compute(image1, keypoints_1, descriptors_1);
    descriptor->compute(image2, keypoints_2, descriptors_2);
    
    // Step3: feature match
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    
    // Step4: feature optimization
    auto min_max = std::minmax_element(matches.begin(), matches.end()); //, [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    
    std::cout << "min_dist: " << min_dist << std::endl;
    std::cout << "max dist: " << max_dist << std::endl;
    
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }
    
    // Step5: show result and comapre with
    cv::Mat img_match, img_goodmatch;
    cv::drawMatches(image1, keypoints_1, image2, keypoints_2, matches, img_match);
    cv::drawMatches(image1, keypoints_1, image2, keypoints_2, good_matches, img_goodmatch);
    
    cv::imshow("all matches", img_match);
    cv::imshow("good amtches", img_goodmatch);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
