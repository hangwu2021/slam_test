#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>


void find_feature_matches(
    const cv::Mat &img_1, 
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);

cv::Point2d pixel2cam(
    const cv::Point2d &p,
    const cv::Mat &K
);

// BA by G2O
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const cv::Mat &K,
    Sophus::SE3d &pose
);



/**********************************
 * ** Main Function *****
 **********************************/
int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        std::cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << std::endl;
        
        return 1;
    }
    
    cv::Mat img_1 = cv::imread(argv[1]), img_2 = cv::imread(argv[2]);
    assert(img_1.data != nullptr && img_2.data != nullptr);
    
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    
    // Feature Match
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "Total Matched = " << matches.size() << " DaulPoints." << std::endl;
    
    cv::Mat d1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    for (cv::DMatch m : matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
            continue;
        float dd = d / 5000.0;
        cv::Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::Mat r, t;
    cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false); 
    cv::Mat R;
    cv::Rodrigues(r, R); 
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << std::endl;

    std::cout << "R=" << std::endl << R << std::endl;
    std::cout << "t=" << std::endl << t << std::endl;

    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }
    
    std::cout << "calling bundle adjustment by g2o" << std::endl;
    Sophus::SE3d pose_g2o;
    t1 = std::chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << std::endl;
    
    
    return 0;
}


void find_feature_matches(
    const cv::Mat &img_1, 
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    cv::Mat descriptors_1, descriptors_2;
    
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
    
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    
    std::vector<cv::DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);
    
    double min_dist = 10000, max_dist = 0;
    for (auto &m : match)
    {
        if (m.distance < min_dist)
        {
            min_dist = m.distance;
        }
        
        if (m.distance > max_dist)
        {
            max_dist = m.distance;
        }
    }
    std::cout << "min_dist = " << min_dist << std::endl;
    std::cout << "max_dist = " << max_dist << std::endl;
    
    for (auto &m : match)
    {
        if (m.distance <= std::max(2*min_dist, 30.0))
        {
            matches.push_back(m);
        }
    }
}

cv::Point2d pixel2cam(
    const cv::Point2d &p,
    const cv::Mat &K
)
{
    return cv::Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

// BA with G2O
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }
    
    virtual void oplusImpl(const number_t * update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
    
    virtual bool read(std::istream &in) override {}
    
    virtual bool write(std::ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}
    
    virtual void computeError() override 
    {
        const VertexPose *v = static_cast<VertexPose *>( _vertices[0] );
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }
    
    virtual void linearizeOplus() override
    {
        const VertexPose *v = static_cast<VertexPose *>( _vertices[0] );
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z, 
                        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }
    
    virtual bool read(std::istream &in) override {}
    
    virtual bool write(std::ostream &out) const override {}
    
private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const cv::Mat &K,
    Sophus::SE3d &pose
)
{
    // Constructing Graph
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
    );
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    
    // Vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);
    
    // K
    Eigen::Matrix3d K_eigen;
    K_eigen <<  K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
                K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
    
    // add Edge
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i)
    {
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        
        optimizer.addEdge(edge);
        index++;
    }
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    
    std::cout << "optimization_costs time: " << time_used.count() << " seconds." << std::endl;
    std::cout << "pose estimated by g2o = \n" << vertex_pose->estimate().matrix() << std::endl;
    
    pose = vertex_pose->estimate();
}
