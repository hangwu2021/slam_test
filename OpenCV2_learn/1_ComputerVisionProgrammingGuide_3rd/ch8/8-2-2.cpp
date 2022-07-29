#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


class HarrisDetector
{
public:
    HarrisDetector() : neighborhood(3), aperture(3), k(0.01), maxStrength(0.0), threshold(0.01), nonMaxSize(3)
    {
        
    }
    
    // Compute Harris corner
    void detect(const cv::Mat &image)
    {
        cv::cornerHarris(image, cornerStrength, neighborhood, aperture, k);
        cv::minMaxLoc(cornerStrength, 0, &maxStrength);
        
        cv::Mat dilated;
        cv::dilate(cornerStrength, dilated, cv::Mat());
        cv::compare(cornerStrength, dilated, localMax, cv::CMP_EQ);
    }
    
    // Corner Distribution Graph
    cv::Mat getCornerMap(double qualityLevel)
    {
        cv::Mat cornerMap;
        threshold = qualityLevel * maxStrength;
        cv::threshold(cornerStrength, cornerTh, threshold, 255, cv::THRESH_BINARY);
        
        cornerTh.convertTo(cornerMap, CV_8U);
        
        cv::bitwise_and(cornerMap, localMax, cornerMap);
        
        return cornerMap;
    }
    
    void getCorners(std::vector<cv::Point> &points, double qualityLevel)
    {
        cv::Mat cornerMap = getCornerMap(qualityLevel);
        
        getCorners(points, cornerMap);
    }
    
    void getCorners(std::vector<cv::Point> &points, const cv::Mat &cornerMap)
    {
        for (int y = 0; y < cornerMap.rows; y++)
        {
            const uchar *rowPtr = cornerMap.ptr<uchar>(y);
            for (int x = 0; x < cornerMap.cols; x++)
            {
                if (rowPtr[x])
                {
                    points.push_back(cv::Point(x, y));
                }
            }
        }
    }
    
    void drawOnImage(
        cv::Mat &image,
        const std::vector<cv::Point> &points,
        cv::Scalar color = cv::Scalar(255, 255, 255),
        int radius = 6,
        int thickness = 1
    )
    {
        std::vector<cv::Point>::const_iterator it = points.begin();
        while (it != points.end())
        {
            cv::circle(image, *it, radius, color, thickness);
            ++it;
        }
    }
    
private:
    cv::Mat cornerStrength;     // bits: 32
    cv::Mat cornerTh;           // bits: 32, Threshold
    cv::Mat localMax;
    int     neighborhood;
    int     aperture;
    double  k;                  // Harris Parameters
    double  maxStrength;
    double  threshold;
    int     nonMaxSize;
    cv::Mat kernel;
};


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    cv::Mat image = cv::imread(argv[1], 0);
    assert(image.data != nullptr);
    
    cv::resize(image, image, cv::Size(image.rows/3, image.cols/3));
    
    HarrisDetector harris;
    harris.detect(image);
    
    std::vector<cv::Point> pts;
    harris.getCorners(pts, 0.02);
    
    harris.drawOnImage(image, pts);
    
    cv::imshow("image", image);
    
    // Way 2: GFTT Detector
    cv::Mat image2 = cv::imread(argv[1], 0);
    cv::resize(image2, image2, cv::Size(image2.rows / 3, image2.cols/3));
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::GFTTDetector> ptrGFTT = cv::GFTTDetector::create(500, 0.01, 10);
    ptrGFTT->detect(image2, keypoints);
    
    std::vector<cv::Point> points;
    for (auto &p : keypoints)
    {
        points.push_back(cv::Point(p.pt.x, p.pt.y));
    }
    
    harris.drawOnImage(image2, points);
    cv::imshow("image2", image2);
    
    cv::waitKey(0);
    
    return 0;
}


