#ifndef VIDEOPROCESSOR_H_
#define VIDEOPROCESSOR_H_

#include "FrameProcessor.h"

class VideoProcessor 
{
public:
    void setFrameProcessor(void (*frameProcessingCallback)(cv::Mat &, cv::Mat &))
    {
        process = frameProcessingCallback;
    }
    
    bool setInput(std::string filename)
    {
        fnumber = 0;
        capture.release();
        
        return capture.open(filename);
    }
    
    bool setOutput(const std::string &filename, int codec=0, double framerate=0.0, bool isColor=true)
    {
        outputFile = filename;
        
        if (framerate == 0.0)
        {
            framerate = getFrameRate();
        }
        
        char c[4];
        if (codec == 0)
        {
            codec = getCodec(c);
        }
        
        //isColor = false;
        
        return writer.open(outputFile, CV_FOURCC('M', 'J', 'P', 'G'), framerate, getFrameSize(), isColor);
    }
    
    void displayInput(std::string wn)
    {
        windowNameInput = wn;
        cv::namedWindow(windowNameInput);
    }
    
    void displayOutput(std::string wn)
    {
        windowNameOutput = wn;
        cv::namedWindow(windowNameOutput);
    }
    
    void run()
    {
        cv::Mat frame;
        cv::Mat output;
        
        if (!isOpened())
        {
            return;
        }
        
        stop = false;
        while (!isStopped())
        {
            if (!readNextFrame(frame))
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
                fnumber++;
            }
            else 
            {
                output = frame;
            }
            
            // Write Frame to Output File
            /*if (outputFile.length() != 0)
            {
            //    writeNextFrame(output);
            }*/
            
            if (windowNameOutput.length() != 0)
            {
                cv::imshow(windowNameOutput, output);
            }
            
            if (delay >= 0 && cv::waitKey(delay) >= 0)
            {
                stopIt();
            }
            
            if (frameToStop >= 0 && getFrameNumber() == frameToStop)
            {
                stopIt();
            }
        }
    }
    
    void stopIt()
    {
        stop = true;
    }
    
    bool isStopped()
    {
        return stop;
    }
    
    bool isOpened()
    {
        return capture.isOpened();
    }
    
    void setDelay(int d)
    {
        delay = d;
    }

private:
    bool readNextFrame(cv::Mat &frame)
    {
        return capture.read(frame);
    }
    
    void writeNextFrame(cv::Mat &frame)
    {
        writer.write(frame);
    }
    
public:
    void callProcess()
    {
        callIt = true;
    }
    
    void dontCallProcess()
    {
        callIt = false;
    }
    
    void stopAtFrameNo(long frame)
    {
        frameToStop = frame;
    }
    
    long getFrameNumber()
    {
        long fnumber = static_cast<long>(capture.get(CV_CAP_PROP_POS_FRAMES));
        
        return fnumber;
    }
    
    double getFrameRate()
    {
        return capture.get(CV_CAP_PROP_FPS);
    }
    
    cv::Size getFrameSize()
    {
        return cv::Size(
            capture.get(CV_CAP_PROP_FRAME_HEIGHT),
            capture.get(CV_CAP_PROP_FRAME_WIDTH)  
        );
    }
    
    int getCodec(char codec[4])
    {
        union {
            int value;
            char code[4];
        } returned;
        
        returned.value = static_cast<int>(capture.get(cv::CAP_PROP_FOURCC));
        
        codec[0] = returned.code[0];
        codec[1] = returned.code[1];
        codec[2] = returned.code[2];
        codec[3] = returned.code[3];
        
        return returned.value;
    }
    
    
    
private:
    cv::VideoCapture capture;
    void (*process)(cv::Mat &frame, cv::Mat &output);
    bool callIt;
    std::string windowNameInput;
    std::string windowNameOutput;
    int delay;
    long fnumber;
    long frameToStop;
    bool stop;
    
private:
    cv::VideoWriter writer;
    std::string outputFile;
};

#endif // VIDEOPROCESSOR_H_
