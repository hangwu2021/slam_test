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
    
    // Grid ROI
    int vsize = 78, hsize = 92;
    int vstep = image.rows / vsize, hstep = image.cols / hsize;
    
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::KeyPoint> gridpoints;
    cv::Mat imageROI;
    cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create(40);
    int subTotal = 50;
    for (int i = 0; i < vstep; i++)
    {
        for (int j = 0; j < hstep; j++)
        {
            imageROI = image(cv::Rect(j*hsize, i*vsize, hsize, vsize));
            
            // Detecting KeyPoint
            gridpoints.clear();
            ptrFAST->detect(imageROI, gridpoints);
            
            auto itEnd(gridpoints.end());
            if (gridpoints.size() > subTotal)
            {
                std::nth_element(gridpoints.begin(), gridpoints.begin()+subTotal, gridpoints.end(), [](cv::KeyPoint &a, cv::KeyPoint &b) { return a.response > b.response; });
                itEnd = gridpoints.begin() + subTotal;
            }
            
            // Add grid keypoints to Global Vector
            for (auto it = gridpoints.begin(); it != itEnd; ++it)
            {
                it->pt += cv::Point2f(j*hsize, i*vsize);
                keypoints.push_back(*it);
            }
        }
    }
    
    cv::drawKeypoints(image, keypoints, image, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    
    cv::imshow("image", image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
