#include "FeatureTracker.h"

void FeatureTracker::process(cv::Mat & frame, cv::Mat & output)
{
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);
    
    // 1. add new point 
    if (this->addNewPoints())
    {
        this->detectFeaturePoints();
        points[0].insert(points[0].end(), features.begin(), features.end());
        initial.insert(initial.end(), features.begin(), features.end());
    }
    
    if (gray_prev.empty())
    {
        gray.copyTo(gray_prev);
    }
    
    // 2. Tracking Feature Points
    cv::calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
    
    // 3. Erase Points
    int k = 0;
    for (int i = 0; i < points[1].size(); ++i)
    {
        if (this->acceptTrackedPoint(i))
        {
            initial[k] = initial[i];
            points[1][k++] = points[1][i];
        }
    }
    
    points[1].resize(k);
    initial.resize(k);
    
    // 4. handle points accepted
    this->handleTrackedPoints(frame, output);
    
    // 5. swap
    std::swap(points[1], points[0]);
    cv::swap(gray_prev, gray);
    
    
}

bool FeatureTracker::addNewPoints()
{
    return points[0].size() <= 10;
}

void FeatureTracker::detectFeaturePoints()
{
    cv::goodFeaturesToTrack(gray, features, max_count, qlevel, minDist);
}

bool FeatureTracker::acceptTrackedPoint(int i)
{
    return status[i] && ((std::abs(points[0][i].x - points[1][i].x) + std::abs(points[0][i].y - points[1][i].y)) > 2);
}

void FeatureTracker::handleTrackedPoints(cv::Mat& frame, cv::Mat& output)
{
    for (int i = 0; i < points[1].size(); ++i)
    {
        cv::line(output, initial[i], points[1][i], cv::Scalar(255, 255, 255));
        cv::circle(output, points[1][i], 3, cv::Scalar(255, 255, 255), -1);
    }
}

void FeatureTracker::drawOpticalFlow(const cv::Mat& oflow, cv::Mat& flowImage, int stride, float scale, const cv::Scalar& color)
{
    if (flowImage.size() != oflow.size())
    {
        flowImage.create(oflow.size(), CV_8UC3);
        flowImage = cv::Vec3i(255, 255, 255);
    }
    
    for (int y = 0; y < oflow.rows; y += stride)
    {
        for (int x = 0; x < oflow.cols; x += stride)
        {
            cv::Point2f vector = oflow.at<cv::Point2f>(y, x);
            
            cv::line(flowImage, cv::Point(x, y), cv::Point(static_cast<int>(x + scale * vector.x + 0.5), static_cast<int>(y + scale * vector.y + 0.5)), color);
            cv::circle(flowImage, cv::Point(static_cast<int>(x + scale*vector.x + 0.5), static_cast<int>(y+scale*vector.y + 0.5)), 1, color, -1);
        }
    }
}

