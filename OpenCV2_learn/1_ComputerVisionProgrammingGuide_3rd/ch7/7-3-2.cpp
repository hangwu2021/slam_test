#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class LineFinder
{
public:
    LineFinder() : deltaRho(1), deltaTheta(CV_PI/180), minVote(10), minLength(0.0), maxGap(0.0)
    {
        
    }
    
    void setAccResolution(double dRho, double dTheta)
    {
        deltaRho = dRho;
        deltaTheta = dTheta;
    }
    
    void setMinVote(int minv)
    {
        this->minVote = minv;
    }
    
    void setLineLengthAndGap(double length, double gap)
    {
        this->minLength = length;
        this->maxGap = gap;
    }
    
    std::vector<cv::Vec4i> findLines(cv::Mat &binary)
    {
        lines.clear();
        
        cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
        
        return this->lines;
    }
    
    void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255, 255, 255)) // 
    {
        std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
        while (it2 != lines.end())
        {
            cv::Point pt1((*it2)[0], (*it2)[1]);
            cv::Point pt2((*it2)[2], (*it2)[3]);
            
            cv::line(image, pt1, pt2, color, 2);
            
            ++it2;
        }
    }
    
private:
    cv::Mat                 img;
    std::vector<cv::Vec4i>  lines;
    double                  deltaRho;
    double                  deltaTheta;
    int                     minVote;
    double                  minLength;
    double                  maxGap;
};


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    cv::imshow("image", image);
    // Canny First
    cv::Mat contours;
    cv::Canny(image, contours, 125, 350);
    
    // Hough Lines
    LineFinder finder;
    finder.setLineLengthAndGap(100, 20);
    finder.setMinVote(60);
    
    std::vector<cv::Vec4i> lines = finder.findLines(contours);
    finder.drawDetectedLines(image);
    
    cv::imshow("linesImage", image);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
