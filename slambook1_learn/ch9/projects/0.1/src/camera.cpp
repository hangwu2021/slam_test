#include "myslam/camera.h"

namespace myslam
{

Camera::Camera()
{
    
}
    
Camera::Camera(float fx, float fy, float cx, float cy, float depth_scale) 
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale)
{
    
}

Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d& p_w, const Sophus::SE3d& T_c_w)
{
    return T_c_w * p_w;
}

Eigen::Vector3d Camera::camera2wolrd(const Eigen::Vector3d& p_c, const Sophus::SE3d& T_c_w)
{
    return T_c_w.inverse() * p_c;
}

Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d& p_c)
{
    return Vector2d(
        fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
        fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d& p_p, double depth)
{
    return Vector3d(
        (p_p(0, 0) - cx_) * depth / fx_,
        (p_p(1, 0) - cy_) * depth / fy_,
        depth
    );
}

Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d& p_w, const Sophus::SE3d& T_c_w)
{
    return camera2pixel(world2camera(p_w, T_c_w));
}

Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d& p_p, const Sophus::SE3d& T_c_w, double depth)
{
    return camera2wolrd(pixel2camera(p_p, depth), T_c_w);
}

}
