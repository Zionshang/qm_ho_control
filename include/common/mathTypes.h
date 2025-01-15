#pragma once
#include <Eigen/Dense>

#define ROBOTNQ 25
#define ROBOTNV 24

/************************ Vector ***************************/
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::VectorXd;
using Vector12d = typename Eigen::Matrix<double, 12, 1>;
using Vector6d = typename Eigen::Matrix<double, 6, 1>;
using Matrix34d = typename Eigen::Matrix<double, 3, 4>;
using Quaternion = typename Eigen::Matrix<double, 4, 1>;

// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// Quaternion  [x,y,z,w]
using Quat = typename Eigen::Matrix<double, 4, 1>;

// 3x1 Vector
using Vec3 = typename Eigen::Matrix<double, 3, 1>;

// 4x1 Int type Vector
using Vec4 = typename Eigen::Matrix<double, 4, 1>;

// 4x1 Int type Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

// 6x1 Vector
using Vec6 = typename Eigen::Matrix<double, 6, 1>;

// 12x1 Vector
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

// 18x1 Vector
using Vec18 = typename Eigen::Matrix<double, 18, 1>;

// Generalized Vetor
using VecNq = typename Eigen::Matrix<double, ROBOTNQ, 1>;
using VecNv = typename Eigen::Matrix<double, ROBOTNV, 1>;

// Dynamic Length Vector
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************ Matrix ***************************/

// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;

// 3x3 Matrix
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

// 3x3 Identity Matrix
#define eye3 Eigen::MatrixXd::Identity(3, 3)

// 2x4 Matrix, each column is a 3x1 vector
using Vec24 = typename Eigen::Matrix<double, 2, 4>;

// 3x4 Matrix, each column is a 3x1 vector
using Vec34 = typename Eigen::Matrix<double, 3, 4>;

// 6x6 Matrix
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

// jacobian
using Jacb = typename Eigen::Matrix<double, 6, ROBOTNV>;
using JacbP = typename Eigen::Matrix<double, 3, ROBOTNV>;

// nv x nv Matrix
using MatNv = typename Eigen::Matrix<double, ROBOTNV, ROBOTNV>;

// Dynamic Size Matrix
using MatX = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatXrow = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// DiagonalMatrix
using Diag3 = typename Eigen::DiagonalMatrix<double, 3>;
using Diag6 = typename Eigen::DiagonalMatrix<double, 6>;

/************************ Function ***************************/
// convert Vec34 to Vec12
inline Vec12 vec34ToVec12(const Vec34 &vec34)
{
    Vec12 vec12;
    for (int i = 0; i < 4; i++)
        vec12.segment(3 * i, 3) = vec34.col(i);
    return vec12;
}

// convert Vec34 to Vec12
inline Vec34 vec12ToVec34(const Vec12 &vec12)
{
    Vec34 vec34;
    for (int i = 0; i < 4; i++)
    {
        vec34.col(i) = vec12.segment(3 * i, 3);
    }
    return vec34;
}
