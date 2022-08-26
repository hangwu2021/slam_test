/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
    class Frame 
    {
    public:
        typedef std::shared_ptr<Frame>  Ptr;
        unsigned long                   id_;
        double                          time_stamp_;
        SE3d                            T_c_w_;
        Camera::Ptr                     camera_;
        cv::Mat                         color_, depth_;
        
    public:
        Frame();
        Frame(long id, double time_stamp=0, SE3d T_c_w=SE3d(), Camera::Ptr camera = nullptr, cv::Mat color=cv::Mat(), cv::Mat depth=cv::Mat());
        ~Frame();
        
        static Frame::Ptr createFrame();
        
        double findDepth(const cv::KeyPoint& kp);
        Vector3d getCamCenter() const;
        bool isInFrame(const Vector3d& pt_world);
    };
}

#endif  // FRAME_H_
