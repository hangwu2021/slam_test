#include "config.hpp"
#include "visual_odometry.hpp"

VisualOdometry::VisualOdometry()
{
    state_ = INITIALIZING;
    map_ = Map::Ptr(new Map);
    ref_ = nullptr;
    curr_ = nullptr;
    num_inliers_ = 0;
    num_lost_ = 0;
    
    num_inliers_ = Config::get<int>("number_of_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<int>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_translation");
    
    //orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    detector_ = cv::ORB::create();//(num_of_features_, scale_factor_, level_pyramid_);
    descriptor_ = cv::ORB::create();//(num_of_features_, scale_factor_, level_pyramid_);
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

VisualOdometry::~VisualOdometry()
{
    
}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch(state_)
    {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            
            map_->insertKeyFrame(frame);
            
            extractKeyPoints();
            computeDescriptors();
            setRef3DPoints();
            
            break;
        }
        
        case OK:
        {
            curr_ = frame;
            
            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();
            std::cout << T_c_r_estimated_.rotationMatrix() << std::endl;
            
            if (checkEstimatedPose() == true)
            {
                curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;    // T_c_w = T_c_r * T_c_w;
                ref_ = curr_;
                setRef3DPoints();
                num_lost_ = 0;
                if (checkKeyFrame() == true)
                {
                    addKeyFrame();
                }
            }
            else 
            {
                num_lost_++;
                if (num_lost_ > max_num_lost_)
                {
                    state_ = LOST;
                }
                
                return false;
            }
            
            break;
        }
        
        case LOST:
        {
            std::cout << "vo has lost." << std::endl;
            break;
        }
    }
    
    return true;
}

void VisualOdometry::extractKeyPoints()
{
    //orb_->detect(curr_->color_, keypoints_curr_);
    //cv::imshow("keys", curr_->color_);cv::waitKey(0);
    detector_->detect(curr_->color_, keypoints_curr_);
    std::cout << "keypoints_curr_.size() = " << keypoints_curr_.size() << std::endl;
}

void VisualOdometry::computeDescriptors()
{
    //orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
    detector_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
}

void VisualOdometry::featureMatching()
{
    std::vector<cv::DMatch> matches;
    //cv::BFMatcher matcher(cv::NORM_HAMMING);
    //matcher.match(descriptors_ref_, descriptors_curr_, matches);
    matcher_->match(descriptors_ref_, descriptors_curr_, matches);
    
    float min_dist = std::min_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2){return m1.distance < m2.distance;})->distance;
    
    feature_matches_.clear();
    for (cv::DMatch& m : matches)
    {
        if (m.distance < std::max<float>(min_dist*match_ratio_, 30.0))
        {
            feature_matches_.push_back(m);
        }
    }
    
    std::cout << "good matches: " << feature_matches_.size() << std::endl;
}

void VisualOdometry::setRef3DPoints()
{
    pts_3d_ref_.clear();
    descriptors_ref_ = cv::Mat();
    for (size_t i = 0; i < keypoints_curr_.size(); i++)
    {
        double d = ref_->findDepth(keypoints_curr_[i]);
        if (d > 0)
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            
            pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

void VisualOdometry::poseEstimationPnP()
{
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;
    for (cv::DMatch& m : feature_matches_)
    {
        pts3d.push_back(pts_3d_ref_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }
    
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0, 0, 1
    );
    std::cout << "pts3d.size() = " << pts3d.size() << ", pts2d.size() = " << pts2d.size() << std::endl;
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    std::cout << "pnp inliers: " << num_inliers_ << std::endl;
    
    cv::Mat RR;
    cv::Rodrigues(rvec, RR);
    
    Eigen::Matrix3d Rvec;
    Rvec << RR.at<double>(0, 0), RR.at<double>(0, 1), RR.at<double>(0, 2), 
            RR.at<double>(1, 0), RR.at<double>(1, 1), RR.at<double>(1, 2), 
            RR.at<double>(2, 0), RR.at<double>(2, 1), RR.at<double>(2, 2);
    
    //Eigen::Quaterniond q(Rvec);
    SO3d SO3d_R(Rvec);
    
    T_c_r_estimated_ = SE3d(
        SO3d_R, 
        //SO3d(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
    );
}

bool VisualOdometry::checkEstimatedPose()
{
    if (num_inliers_ < min_inliers_)
    {
        std::cout << "inliers is too small: " << num_inliers_ << std::endl;
        return false;
    }
    
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if (d.norm() > 5.0)
    {
        std::cout << "motion is too large: " << d.norm() << std::endl;
        return false;
    }
    
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
    {
        return true;
    }
    
    return false;
}

void VisualOdometry::addKeyFrame()
{
    std::cout << "adding a key-frame" << std::endl;
    map_->insertKeyFrame(curr_);
}
