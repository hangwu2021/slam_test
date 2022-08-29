#include "camera.hpp"
#include "config.hpp"

Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}

Eigen::Vector2d  Camera::camera2pixel(const Eigen::Vector3d& p_c)
{
    return Eigen::Vector2d(
        fx_ * (p_c(0, 0) / p_c(2, 0)) + cx_,
        fy_ * (p_c(1, 0) / p_c(2, 0)) + cy_
    );
}
