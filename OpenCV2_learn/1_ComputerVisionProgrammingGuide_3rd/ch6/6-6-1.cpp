#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// def LaplacianZC
class LaplacianZC
{
public:
    LaplacianZC() : aperture(3) 
    {
        
    }
    
    void setAperture(int a)
    {
        aperture = a;
    }
    
    cv::Mat computeLaplacian(const cv::Mat& image)
    {
        cv::Laplacian(image, laplace, CV_32F, aperture);
        
        return laplace;
    }
    
    cv::Mat getLaplacianImage(double scale = -1.0)
    {
        if (scale < 0)
        {
            double lapmin, lapmax;
            cv::minMaxLoc(laplace, &lapmin, &lapmax);
            scale = 127 / std::max(-lapmin, lapmax);
        }
        
        cv::Mat laplaceImage;
        
        laplace.convertTo(laplaceImage, CV_8U, scale, 128);
        
        return laplaceImage;
    }
    
    cv::Mat getZeroCrossings(cv::Mat laplaceSrc)
    {
        cv::Mat signImage;
        cv::threshold(laplaceSrc, signImage, 0, 255, cv::THRESH_BINARY);
        
        cv::Mat binary;
        signImage.convertTo(binary, CV_8U);
        
        cv::Mat dilated;
        cv::dilate(binary, dilated, cv::Mat());
        
        return (dilated - binary);
    }
    
private:
    cv::Mat     laplace;
    int         aperture;
};

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Parameters Error!" << std::endl;
        return -1;
    }
    
    // Instance LaplacianZC
    LaplacianZC laplacian;
    
    // Load Gray Image
    cv::Mat image = cv::imread(argv[1], 0);
    
    cv::Mat flap = laplacian.computeLaplacian(image);
    
    cv::imshow("laplace", flap);
    
    cv::Mat laplace = laplacian.getLaplacianImage();
    cv::imshow("laplaceImg", laplace);
    
    cv::Mat dbImage;
    dbImage = laplacian.getZeroCrossings(flap);
    cv::imshow("ZeroCross", dbImage);
    
    // DoG(Difference of Gaussians, DoG)
    cv::Mat gauss20, gauss22, dog, zeros;
    cv::GaussianBlur(image, gauss20, cv::Size(), 2.0);
    cv::GaussianBlur(image, gauss22, cv::Size(), 2.2);
    cv::subtract(gauss22, gauss20, dog, cv::Mat(), CV_32F);
    
    zeros = laplacian.getZeroCrossings(dog);
    
    cv::imshow("dog", zeros);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
