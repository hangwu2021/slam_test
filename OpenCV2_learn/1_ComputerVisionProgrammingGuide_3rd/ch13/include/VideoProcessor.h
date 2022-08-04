#ifndef VIDEOPROCESSOR_H_
#define VIDEOPROCESSOR_H_

#include "FrameProcessor.h"

class VideoProcessor
{
public:
    void setFrameProcessor(void (*frameProcessorCallback)(cv::Mat &, cv::Mat &));
    bool setInput(std::string filename);
    void displayInput(std::string wn);
    void displayOutput(std::string wn);
    void run();
    void stopIt();
    bool isStopped();
    bool isOpened();
    void setDelay(int d);
    
private:
    bool readNextFrame(cv::Mat &frame);
    
public:
    void callProcess();
    void dontCallProcess();
    void stopAtFrameNo(long frame);
    long getFrameNumber();
    int getFrameRate();
    
private:
    cv::VideoCapture capture;
    void (*process)(cv::Mat &, cv::Mat &);
    bool callIt;
    std::string windowNameInput;
    std::string windowNameOutput;
    int delay;
    long fnumber;
    long frameToStop;
    bool stop;
};

#endif  // VIDEOPROCESSOR_H_
