#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image1 = cv::imread(argv[1]), image2 = cv::imread(argv[2]);
    assert(image1.data != nullptr && image2.data != nullptr);
    
    //MatchTemplate
    cv::Ptr<cv::FeatureDetector> ptrDetector;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    
    ptrDetector = cv::FastFeatureDetector::create(80);
    ptrDetector->detect(image1, keypoints1);
    ptrDetector->detect(image2, keypoints2);
    
    // define Match Template
    const int nsize(11);
    cv::Rect neighborhood(0, 0, nsize, nsize);
    cv::Mat patch1, patch2;
    cv::Mat result;
    std::vector<cv::DMatch> matches;
    
    for (int i = 0; i < keypoints1.size(); i++)
    {
        neighborhood.x = keypoints1[i].pt.x - nsize/2;
        neighborhood.y = keypoints1[i].pt.y - nsize/2;
        
        if (neighborhood.x < 0 || neighborhood.y < 0 || neighborhood.x + nsize >= image1.cols || neighborhood.y + nsize >= image1.rows)
        {
            continue;
        }
        
        patch1 = image1(neighborhood);
        
        cv::DMatch bestMatch;
        for (int j = 0; j < keypoints2.size(); j++)
        {
            neighborhood.x = keypoints2[j].pt.x - nsize/2;
            neighborhood.y = keypoints2[j].pt.y - nsize/2;
            
            if (neighborhood.x < 0 || neighborhood.y < 0 || neighborhood.x + nsize >= image2.cols || neighborhood.y + nsize >= image2.rows)
            {
                continue;
            }
            
            patch2 = image2(neighborhood);
            
            cv::matchTemplate(patch1, patch2, result, cv::TM_SQDIFF);
            
            if (result.at<float>(0, 0) < bestMatch.distance)
            {
                bestMatch.distance = result.at<float>(0, 0);
                bestMatch.queryIdx = i;
                bestMatch.trainIdx = j;
            }
        }
        
        matches.push_back(bestMatch);
    }
    
    const int N = 25;
    std::nth_element(matches.begin(), matches.begin() + N, matches.end());
    
    matches.erase(matches.begin() + 25, matches.end());
    
    cv::Mat matchImage;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));
    
    cv::imshow("matchImage", matchImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}





