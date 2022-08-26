#include "config.hpp"
#include "camera.hpp"
#include "visual_odometry.hpp"
#include <opencv2/viz.hpp>
//#include <opencv2/viz/viz3d.hpp>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./bin/run_vo config/default.yaml" << std::endl;
        return -1;
    }
    
    Config::setParameterFile( argv[1] );
    
    VisualOdometry::Ptr vo( new VisualOdometry);
    
    std::string dataset_dir = Config::get<std::string>("dataset_dir");
    std::cout << "dataset_dir" << dataset_dir << std::endl;
    std::ifstream fin(dataset_dir+"/associate.txt");
    if (!fin)
    {
        std::cout << "associate.txt is not generated." << std::endl;
        return -1;
    }
    
    std::vector<std::string> rgb_files, depth_files;
    std::vector<double> rgb_times, depth_times;
    while(!fin.eof())
    {
        std::string rgb_time, rgb_file, depth_time, depth_file;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        
        rgb_times.push_back( std::atof(rgb_time.c_str()) );
        rgb_files.push_back( dataset_dir + "/" + rgb_file );
        
        depth_times.push_back( std::atof(depth_time.c_str()) );
        depth_files.push_back( dataset_dir + "/" + depth_file );
        
        if (fin.good() == false)
        {
            break;
        }
    }
    
    Camera::Ptr camera( new Camera );
    
    // Visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("world", world_coor);
    vis.showWidget("camera", camera_coor);
    
    std::cout << "read total " << rgb_files.size() << " entries." << std::endl;
    for (int i = 0; i < rgb_files.size(); i++)
    {
        cv::Mat color = cv::imread(rgb_files[i]);
        cv::Mat depth = cv::imread(depth_files[i], -1);
        if (color.data == nullptr || depth.data == nullptr)
        {
            break;
        }
        
        Frame::Ptr pFrame = Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];
        
        vo->addFrame(pFrame);
        
        if (vo->state_ == VisualOdometry::LOST)
        {
            break;
        }
        
        SE3d Tcw = pFrame->T_c_w_.inverse();
        
        cv::Affine3f pose(
            cv::Affine3f::Mat3(
                Tcw.rotationMatrix()(0, 0), Tcw.rotationMatrix()(0, 1), Tcw.rotationMatrix()(0, 2), 
                Tcw.rotationMatrix()(1, 0), Tcw.rotationMatrix()(1, 1), Tcw.rotationMatrix()(1, 2), 
                Tcw.rotationMatrix()(2, 0), Tcw.rotationMatrix()(2, 1), Tcw.rotationMatrix()(2, 2)
            ),
            cv::Affine3f::Vec3(
                Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)
            )
        );
        
        cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
        cv::imshow("image", color);
        cv::waitKey(110);
        
        cv::viz::WCameraPosition cam_pos(0.5);
        vis.showWidget("Camera", cam_pos, pose);
        
        //vis.setWidgetPose("Camera", pose);
        vis.spinOnce(1, false);
    }
    
    return 0;
}
