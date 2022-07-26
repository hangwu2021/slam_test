#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);


int main(int argc, char* argv[])
{
    std::vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType poses;
    
    std::ifstream fin("./pose.txt");
    if (!fin)
    {
        std::cerr << "The Path of pose.txt is not correct!" << std::endl;
        return -1;
    }
    
    for (int i = 0; i < 5; i++)
    {
        boost::format fmt("./%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i+1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i+1) % "pgm").str(), -1));
        
        double data[7] = {0.0};
        for (auto& d: data)
        {
            fin >> d;
        }
        
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                          Eigen::Vector3d(data[0], data[1], data[2])
        );
        
        poses.push_back(pose);
    }
    
    // camera intrics
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);
    
    for (int i = 0; i < 5; i++)
    {
        std::cout << "Image Transforming: " << i+1 << std::endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];
        std::cout << color.rows << ", " << color.cols << std::endl;
        for (int v = 0; v < color.rows; v++)
        {
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if (d == 0)
                {
                    continue;
                }
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                
                Eigen::Vector3d pointworld = T * point;
                
                Vector6d p;
                p.head<3>() = pointworld;
                p[5] = color.data[v * color.step + u * color.channels()];       // B
                p[4] = color.data[v * color.step + u * color.channels() + 1];   // G
                p[3] = color.data[v * color.step + u * color.channels() + 2];   // R
                
                pointcloud.push_back(p);
            }
        }
    }
    
    std::cout << "Total Cloud Point: " << pointcloud.size() << std::endl;
    
    return 0;
}

void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud)
{
    if (pointcloud.empty())
    {
        std::cerr << "Point Cloud is empty!" << std::endl;
        return ;
    }
    
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );
    
}
