#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char* argv[])
{
    // Rotation Matrix
    Matrix3d rotation_matrix = Matrix3d::Identity();
    AngleAxisd rotation_vector(M_PI/4, Vector3d(0, 0, 1));
    std::cout << "rotation matrix = \n" << rotation_vector.matrix() << std::endl;
    
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    std::cout << "(1, 0, 0) after rotation (by angle axis) = \n" << v_rotated.transpose() << std::endl;
    
    v_rotated = rotation_matrix * v;
    std::cout << "(1, 0, 0) after rotation (by matrix) = \n" << v_rotated.transpose() << std::endl;
    
    // Euler Matrix
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);   // ZYX -> roll pitch yaw
    std::cout << "yaw pitch roll = \n" << euler_angles.transpose() << std::endl;
    
    // Euler Matrix
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1, 3, 4));
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;
    
    Vector3d v_transformed = T * v;
    std::cout << "v transformed = " << v_transformed.transpose() << std::endl;
    
    // Quaterniond
    Quaterniond q = Quaterniond(rotation_vector);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;
    
    q = Quaterniond(rotation_matrix);
    std::cout << "quaternion from rotatoin matrix = " << q.coeffs().transpose() << std::endl;
    
    v_rotated = q * v;
    std::cout << "(1, 0, 0) after rotatoin = " << v_rotated.transpose() << std::endl;
    
    std::cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;
    
    return 0;
}
