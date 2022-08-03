
#include "BGFGSegmentor.h"
#include "VideoProcessor.h"


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        return -1;
    }
    
    VideoProcessor processor;
    BGFGSegmentor segmentor;
    
    segmentor.setThreshold(25);
    
    processor.setInput(argv[1])
    processor.setFrameProcessor();
    processor.displayOutput("Extracted Foreground");
    processor.setDelay(1000. / processor.getFrameRate());
    
    processor.run();
    
    return 0;
}
