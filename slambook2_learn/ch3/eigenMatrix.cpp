#include <iostream>
#include <ctime>

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;

#define MATRIX_SIZE 50


int main( int argc, char* argv[])
{
    Matrix<float, 2, 3> matrix_23;
    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;
    Matrix3d matrix_33 = Matrix3d::Zero();
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_x;
    
    matrix_23 << 1, 2, 3, 4, 5, 6;
    
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;
    
    std::cout << "\nprint matrix 2x3: " << std::endl;
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 3; ++j) 
        {
            std::cout << matrix_23(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;
    
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1, 2, 3; 4, 5, 6] * [3, 2, 1]: " << result.transpose() << std::endl;
    
    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    std::cout << "[1, 2, 3; 4, 5, 6] * [4, 5, 6]: " << result2.transpose() << std::endl;
    
    matrix_33 = Matrix3d::Random();
    std::cout << "random matrix:\n" << matrix_33 << std::endl;
    std::cout << "transpose:\n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "times 10: \n" << 10 * matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;
    
    SelfAdjointEigenSolver<Matrix3d> eigen_solver( matrix_33.transpose() * matrix_33 );
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;
    
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();
    
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);
    
    clock_t time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    
    
    
    
    return 0;
}
