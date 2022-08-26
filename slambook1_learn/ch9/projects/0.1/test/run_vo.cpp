//#include "myslam/common_include.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

void svEqualizeHist(const cv::Mat& input, cv::Mat& output);
void svMeanStdBalance(const cv::Mat& input, cv::Mat& output);
void svGammaCorrection(const cv::Mat& input, cv::Mat& output, float gamma);
void svNormalize(const cv::Mat& input, cv::Mat& output);

int main(int argc, char *argv[])
{
    // Load Video
    std::string video_input_path = "data/videos/StitchedVideo_0810-1080x1080.avi";
    cv::VideoCapture video_input(video_input_path);
    if (!video_input.isOpened())
    {
        std::cerr << "Video Load Failed." << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    cv::Mat frame_output = cv::Mat(frame.size(), CV_64FC3);
    while (video_input.read(frame))
    {
        cv::imshow("frame", frame);
        
        //svGammaCorrection(frame, frame_output, 1.5);
        //svEqualizeHist(frame, frame_output);
        
        cv::normalize(frame, frame_output, 0, 255, cv::NORM_MINMAX);
        svEqualizeHist(frame_output, frame_output);
        
        cv::imshow("output", frame_output);
        
        uchar key = cv::waitKey(0);
        if(key == 'Q' || key == 'q')
        {
            break;
        }
    }
    
    video_input.release();
    
    return 0;
}


void svEqualizeHist ( const cv::Mat& input, cv::Mat& output )
{
    cv::Mat input_hsv;
    std::vector<cv::Mat> channels;
    
    cv::cvtColor(input, input_hsv, cv::COLOR_BGR2YUV);
    cv::split(input_hsv, channels);
    
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, input_hsv);
    cv::cvtColor(input_hsv, output, cv::COLOR_YUV2BGR);
}

void svMeanStdBalance ( const cv::Mat& input, cv::Mat& output )
{
    std::cout << input.type() << std::endl;
    double minVal = 0.0, maxVal = 0.0;
    cv::minMaxLoc(input, &minVal, &maxVal);
    
    std::cout << minVal << ", " << maxVal << std::endl;
    
    cv::Mat yuv;
    std::vector<cv::Mat> channels;
    cv::cvtColor(input, yuv, cv::COLOR_BGR2YUV);
    cv::split(input, channels);
    
    cv::Mat mean, stddev;
    cv::meanStdDev(yuv, mean, stddev);
    
    channels[0] = (channels[0] - mean.at<double>(0, 0)) / std::sqrt(stddev.at<double>(0, 0));
    channels[1] = (channels[1] - mean.at<double>(1, 0)) / std::sqrt(stddev.at<double>(1, 0));
    channels[2] = (channels[2] - mean.at<double>(2, 0)) / std::sqrt(stddev.at<double>(2, 0));
    
    cv::merge(channels, yuv);
    cv::cvtColor(yuv, output, cv::COLOR_YUV2BGR);
}

void svGammaCorrection ( const cv::Mat& input, cv::Mat& output, float gamma )
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(std::pow((float)i/255.0, gamma) * 255.0);
    }
    
    output = input.clone();
    cv::MatIterator_<cv::Vec3b> it = output.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> end = output.end<cv::Vec3b>();
    while (it != end)
    {
        (*it)[0] = lut[(*it)[0]];
        (*it)[1] = lut[(*it)[1]];
        (*it)[2] = lut[(*it)[2]];
        it++;
    }
}

void svNormalize(const cv::Mat& input, cv::Mat& output)
{
    cv::Mat tmp = cv::Mat(input.size(), CV_64FC3);
    
    double minVal = 0, maxVal = 0;
    cv::minMaxLoc(input, &minVal, &maxVal);
    
    for (int i = 0; i < input.rows; i++)
    {
        for (int j = 0; j < input.cols; j++)
        {
            tmp.at<cv::Vec3f>(j, i)[0] = (input.at<cv::Vec3b>(j, i)[0] - minVal) / (maxVal - minVal);
            tmp.at<cv::Vec3f>(j, i)[1] = (input.at<cv::Vec3b>(j, i)[1] - minVal) / (maxVal - minVal);
            tmp.at<cv::Vec3f>(j, i)[2] = (input.at<cv::Vec3b>(j, i)[2] - minVal) / (maxVal - minVal);
        }
    }
}

