#pragma once

#include "common_include.hpp"

class Camera 
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float fx_, fy_, cx_, cy_, depth_scale_;
    
    Camera();
    
    Eigen::Vector2d camera2pixel(const Eigen::Vector3d& p_c);
};
