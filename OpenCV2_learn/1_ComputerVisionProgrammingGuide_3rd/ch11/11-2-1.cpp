#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class CameraCalibrator 
{
public:
    int addChessboardPoints(
        const std::vector<std::string> &filelist,
        cv::Size &boardSize
    )
    {
        std::vector<cv::Point2f> imageCorners;
        std::vector<cv::Point3f> objectCorners;
        
        // 3D points
        for (int i = 0; i < boardSize.height; i++)
        {
            for (int j = 0; j < boardSize.width; j++)
            {
                objectCorners.push_back(cv::Point3f(i, j, 0.0f));
            }
        }
        
        // 2D points
        cv::Mat image;
        int successes = 0;
        for (int i = 0; i < filelist.size(); i++)
        {
            image = cv::imread(filelist[i], 0);
            bool found = cv::findChessboardCorners(
                image,
                boardSize,
                imageCorners
            );
            
            if (found)
            {
                cv::cornerSubPix(
                    image,
                    imageCorners,
                    cv::Size(5, 5),
                    cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1)
                );
                
                if (imageCorners.size() == boardSize.area())
                {
                    addPoints();
                }
            }
        }
        
        return successes;
    }
    
private:
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    
    int flag;
};


int main(int argc, char *argv[])
{
    cv::Mat image = cv::imread(argv[1], 0);
    assert(image.data != nullptr);
    
    std::vector<cv::Point2f> imageCorners;
    cv::Size boardSize(9, 6);
    bool found = cv::findChessboardCorners(image, boardSize, imageCorners);
    
    cv::drawChessboardCorners(image, boardSize, imageCorners, found);
    
    cv::imshow("image", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    
    
    
    
    
    return 0;
}
