#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/viz.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using Sophus::SE3d;
using Sophus::SO3d;
