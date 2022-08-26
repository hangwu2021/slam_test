#pragma once

#include "common_include.hpp"
#include "frame.hpp"
#include "mappoint.hpp"

class Map 
{
public:
    typedef std::shared_ptr<Map> Ptr;
    std::unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    std::unordered_map<unsigned long, Frame::Ptr> keyframes_;
    
    Map();
    
    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};
