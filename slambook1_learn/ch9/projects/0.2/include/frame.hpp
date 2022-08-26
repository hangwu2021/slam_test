#pragma once

#include "common_include.hpp"
#include "camera.hpp"

class MapPoint;

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long   id_;
    double          time_stamp_;
    SE3d            T_c_w_;
    Camera::Ptr     camera_;
    cv::Mat         color_, depth_;
    
public:
    Frame();
    Frame(long id, double time_stamp=0, SE3d T_c_w=SE3d(), Camera::Ptr camera=nullptr, cv::Mat color=cv::Mat(), cv::Mat depth=cv::Mat());
    ~Frame();
    
    static Frame::Ptr createFrame();
    
    double findDepth(const cv::KeyPoint& kp);
    
    Vector3d getCamCenter() const;
    
    bool isInFrame(const Vector3d& pt_world);
};
