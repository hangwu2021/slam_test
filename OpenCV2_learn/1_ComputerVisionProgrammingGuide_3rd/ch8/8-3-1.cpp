#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Errors!" << std::endl;
        return -1;
    }
    
    // Load Image
    cv::Mat image = cv::imread(argv[1], 0);
    if (image.data == nullptr)
    {
        std::cerr << "Load Image" << std::endl;
        return -1;
    }
    
    // Source Image is too large to not see beter.
    cv::resize(image, image, cv::Size(image.rows/3, image.cols/3));
    
    // Keypoints Detecting
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create(40);
    ptrFAST->detect(image, keypoints);
    
    int numberOfPoints = 500;
    std::vector<cv::KeyPoint> nth_keypoints;
    if (numberOfPoints < keypoints.size())
    {
        std::nth_element(keypoints.begin(), keypoints.begin()+numberOfPoints, keypoints.end(), [](cv::KeyPoint &a, cv::KeyPoint &b) { return a.response > b.response; });
        for (int i = 0; i < numberOfPoints; i++)
        {
            nth_keypoints.push_back(keypoints[i]);
        }
    }
    
    std::cout << nth_keypoints.size() << std::endl;
    
    cv::drawKeypoints(image, nth_keypoints, image, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //cv::Scalar(-1, -1, -1),
    cv::imshow("FAST", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
