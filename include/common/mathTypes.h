#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/multibody/liegroup/special-orthogonal.hpp>

#define ROBOTNQ 25
#define ROBOTNV 24

/************************ Vector ***************************/
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::VectorXd;
using Eigen::Quaterniond;

using Vector12d = typename Eigen::Matrix<double, 12, 1>;
using Vector6d = typename Eigen::Matrix<double, 6, 1>;
using Matrix34d = typename Eigen::Matrix<double, 3, 4>;

using SO3Group = pinocchio::SpecialOrthogonalOperationTpl<3, double>;

// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 6x1 Vector
using Vector6d = typename Eigen::Matrix<double, 6, 1>;

// Generalized Vetor
using VecNq = typename Eigen::Matrix<double, ROBOTNQ, 1>;
using VecNv = typename Eigen::Matrix<double, ROBOTNV, 1>;


/************************ Matrix ***************************/

// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;


// 2x4 Matrix, each column is a 3x1 vector
using Vec24 = typename Eigen::Matrix<double, 2, 4>;

using Matrix6xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix3xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;

// DiagonalMatrix
using Diagonal3d = typename Eigen::DiagonalMatrix<double, 3>;
using Diagonal6d = typename Eigen::DiagonalMatrix<double, 6>;

/************************ Function ***************************/
// convert Matrix34d to Vector12d
inline Vector12d vec34ToVec12(const Matrix34d &vec34)
{
    Vector12d vec12;
    for (int i = 0; i < 4; i++)
        vec12.segment(3 * i, 3) = vec34.col(i);
    return vec12;
}

// convert Matrix34d to Vector12d
inline Matrix34d vec12ToVec34(const Vector12d &vec12)
{
    Matrix34d vec34;
    for (int i = 0; i < 4; i++)
    {
        vec34.col(i) = vec12.segment(3 * i, 3);
    }
    return vec34;
}
