#include <iostream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>

// G2O
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



// Features Matching
bool findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);


// Camera Inner Parameters
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;


// ************ Main Function **************
int main(int argc, char* argv[]) 
{
    if (argc != 3) 
    {
        std::cout << "Usage: bundle_adjustment_g2o img1 img2" << std::endl;
        return 1;
    }
    
    // Load Image
    cv::Mat img1 = cv::imread(argv[1]);
    cv::Mat img2 = cv::imread(argv[2]);
    
    // Points of Matched
    std::vector<cv::Point2f> points1, points2;
    if ( !findCorrespondingPoints(img1, img2, points1, points2) )
    {
        std::cout << "Count of Points matched is not enghou!" << std::endl;
        return 1;
    }
    
    // g2o Graph
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> (linearSolver) );
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<g2o::BlockSolver_6_3>(block_solver) );
    
    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );
    
    // Add Vertex
    // Step1: Pose Vertex
    for (int i = 0; i < 2; ++i)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        
        v->setId( i );
        if ( i==0 ) 
        {
            v->setFixed(true);
        }
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    
    // Step2: Road Flags
    for ( size_t i = 0; i < points1.size(); ++i) 
    {
        g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
        
        v->setId( 2+i );
        
        double z = 1;   // First define, and then use.
        double x = (points1[i].x - cx) * z / fx;
        double y = (points1[i].y - cy) * z / fy;
        
        v->setMarginalized( true );
        v->setEstimate( Eigen::Vector3d(x, y, z) );
        optimizer.addVertex( v );
    }
    
    // Prepare Camera Parameters
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId( 0 );
    optimizer.addParameter( camera );
    
    // Add Edge
    // Step1: 
    std::vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i = 0; i < points1.size(); ++i)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ*> (optimizer.vertex(2+i)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
        edge->setMeasurement( Eigen::Vector2d(points1[i].x, points1[i].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId( 0, 0 );
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        
        optimizer.addEdge( edge );
        edges.push_back( edge );
    }
    
    // Step2:
    for ( size_t i = 0; i < points2.size(); ++i ) 
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        
        edge->setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*> (optimizer.vertex(2+i)));
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(1)));
        edge->setMeasurement( Eigen::Vector2d(points2[i].x, points2[i].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }
    
    std::cout << "Start Optimizing..." << std::endl;
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize( 10 );
    std::cout << "Optimize Over!" << std::endl;
    
    // Matrix Transpose
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    std::cout << "Pose: \n" << pose.matrix() << std::endl;
    /*
    // Feature Point
    for ( size_t i = 0; i < points1.size(); ++i) 
    {
        g2o::VertexPointXYZ* v = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(2+i));
        std::cout << "vertex id " << 2+i << ", pos = ";// << std::endl;
        Eigen::Vector3d pos = v->estimate();
        std::cout << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;
    }
    
    int inliers = 0;
    for (auto e: edges)
    {
        e->computeError();
        if ( e->chi2() > 1) 
        {
            std::cout << "error = " << e->chi2() << std::endl; 
        }
        else 
        {
            ++inliers;
        }
    }
    
    std::cout << "inliers in total points: " << inliers << "/" << points1.size() + points2.size() << std::endl;
    
    */
    
    //optimizer.save("bundle_adjustment.g2o");
    
    return 0;
}



bool findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    // Step1: detect Oriented FAST Corner Points
    detector->detect(img1, keypoints_1);
    detector->detect(img2, keypoints_2);
    
    // Step2: compute BRIEF descriptors
    descriptor->compute(img1, keypoints_1, descriptors_1);
    descriptor->compute(img2, keypoints_2, descriptors_2);
    
    cv::Mat outimg1;
    cv::drawKeypoints(img1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", outimg1);
    
    
    // Step3: DMatch
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    
    // Step4: optimal match
    auto min_max = std::minmax_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; } );
    double min_dist = min_max.first->distance;
    //double max_dist = min_max.second->distance;
    
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; ++i) 
    {
        if (matches[i].distance <= std::max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
            points1.push_back(keypoints_1[matches[i].queryIdx].pt);
            points2.push_back(keypoints_2[matches[i].trainIdx].pt);
        }
    }
    
    std::cout << "good matches: " << good_matches.size() << std::endl;
    
    if (good_matches.size() < 20) 
    {
        return false;
    }
    
    return true;
}
