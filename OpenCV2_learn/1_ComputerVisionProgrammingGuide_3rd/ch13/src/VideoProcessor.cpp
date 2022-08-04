#include "VideoProcessor.h"

void VideoProcessor::setFrameProcessor(void (*frameProcessorCallback)(cv::Mat &, cv::Mat &))
{
    process = frameProcessorCallback;
}

bool VideoProcessor::setInput(std::string filename)
{
    fnumber = 0;
    
    capture.release();
    
    return capture.open(filename);
}

void VideoProcessor::displayInput(std::string wn)
{
    windowNameInput = wn;
    
    cv::namedWindow(windowNameInput, CV_WINDOW_AUTOSIZE);
}

void VideoProcessor::displayOutput(std::string wn)
{
    windowNameOutput = wn;
    
    cv::namedWindow(windowNameOutput, CV_WINDOW_AUTOSIZE);
}

void VideoProcessor::run()
{
    cv::Mat frame;
    cv::Mat output;
    
    if (! this->isOpened())
    {
        return;
    }
    
    stop = false;
    while (! this->isStopped())
    {
        if (! this->readNextFrame(frame))
        {
            break;
        }
        
        if (windowNameInput.length() != 0)
        {
            cv::imshow(windowNameInput, frame);
        }
        
        if (callIt)
        {
            process(frame, output);
            this->fnumber++;
        }
        else 
        {
            output = frame;
        }
        
        if (windowNameOutput.length() != 0)
        {
            cv::imshow(windowNameOutput, output);
        }
        
        if (this->delay >= 0 && cv::waitKey(delay) >= 0)
        {
            this->stopIt();
        }
    }
}

void VideoProcessor::stopIt()
{
    stop = true;
}

bool VideoProcessor::isStopped()
{
    return stop;
}

bool VideoProcessor::isOpened()
{
    return capture.isOpened();
}

void VideoProcessor::setDelay(int d)
{
    delay = d;
}

bool VideoProcessor::readNextFrame(cv::Mat& frame)
{
    return capture.read(frame);
}

void VideoProcessor::callProcess()
{
    callIt = true;
}

void VideoProcessor::dontCallProcess()
{
    callIt = false;
}

void VideoProcessor::stopAtFrameNo(long frame)
{
    frameToStop = frame;
}

long VideoProcessor::getFrameNumber()
{
    long fnum = static_cast<long>(capture.get(CV_CAP_PROP_POS_FRAMES));
    
    return fnum;
}

int VideoProcessor::getFrameRate()
{
    return this->capture.get(CV_CAP_PROP_FPS);
}

