#include "map.hpp"

Map::Map()
{
    
}

void Map::insertKeyFrame(Frame::Ptr frame)
{
    std::cout << "Key frame size = " << keyframes_.size() << std::endl;
    if (keyframes_.find(frame->id_) == keyframes_.end())
    {
        keyframes_.insert(std::make_pair(frame->id_, frame));
        
    }
    else
    {
        keyframes_[frame->id_] = frame;
    }
    std::cout << __func__ << "key frames size = " << keyframes_.size() << std::endl;
}

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
    if (map_points_.find(map_point->id_) == map_points_.end())
    {
        map_points_.insert(std::make_pair(map_point->id_, map_point));
    }
    else
    {
        map_points_[map_point->id_] = map_point;
    }
}

