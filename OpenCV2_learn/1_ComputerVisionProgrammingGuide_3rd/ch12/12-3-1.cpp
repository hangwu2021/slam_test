#include "VideoProcessor.h"

void canny(cv::Mat &img, cv::Mat &out)
{
    if (img.channels() == 3)
    {
        cv::cvtColor(img, out, cv::COLOR_BGR2GRAY);
    }
    else
    {
        out = img.clone();
    }
    
    cv::Canny(out, out, 100, 200);
    
    cv::threshold(out, out, 128, 255, cv::THRESH_BINARY_INV);
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    VideoProcessor processor;
    
    processor.setInput(argv[1]);
    //processor.setOutput("videoProcessed.avi");
    processor.displayInput("Current Frame");
    processor.displayOutput("Output Frame");
    
    processor.setDelay(1000./processor.getFrameRate());
    processor.setFrameProcessor(canny);
    
    processor.run();
    
    
    return 0;
}
