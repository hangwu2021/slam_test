#pragma once

#include "common_include.hpp"
#include "map.hpp"

class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState         state_;
    Map::Ptr        map_;
    Frame::Ptr      ref_;       // reference frame
    Frame::Ptr      curr_;      // current frame
    
    //cv::Ptr<cv::ORB>            orb_;
    cv::Ptr<cv::FeatureDetector>        detector_;
    cv::Ptr<cv::DescriptorExtractor>    descriptor_;
    cv::Ptr<cv::DescriptorMatcher>      matcher_;
    std::vector<cv::Point3f>    pts_3d_ref_;
    std::vector<cv::KeyPoint>   keypoints_curr_;
    cv::Mat                     descriptors_curr_;
    cv::Mat                     descriptors_ref_;
    std::vector<cv::DMatch>     feature_matches_;
    
    SE3d    T_c_r_estimated_;   // estimated pose of current frame
    int     num_inliers_;
    int     num_lost_;
    
    int     num_of_features_;
    double  scale_factor_;
    int     level_pyramid_;
    float   match_ratio_;
    int     max_num_lost_;
    int     min_inliers_;
    
    double  key_frame_min_rot;      // minimal rotation of two key-frames
    double  key_frame_min_trans;    // minimal translation of two key-frames
    
public:
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame(Frame::Ptr frame);
    
protected:
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();
    
    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();
};
