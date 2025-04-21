#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/multibody/liegroup/special-orthogonal.hpp>
#include <memory>

/************************ Vector ***************************/
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Eigen::VectorXd;

/************************ Matrix ***************************/
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

using Matrix34d = Eigen::Matrix<double, 3, 4>;
using Matrix6xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix3xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;

using Diagonal3d = Eigen::DiagonalMatrix<double, 3>;
using Diagonal6d = Eigen::DiagonalMatrix<double, 6>;

/************************ Other ***************************/
using Eigen::Quaterniond;
using SO3Group = pinocchio::SpecialOrthogonalOperationTpl<3, double>;
using RotMat = Eigen::Matrix<double, 3, 3>;

using std::shared_ptr;