#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
using Sophus::SE3d;
using Sophus::SO3d;

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
