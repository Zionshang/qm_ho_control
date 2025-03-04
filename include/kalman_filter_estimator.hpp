#pragma once
#include "common/types.hpp"
#include "common/low_state.hpp"
#include "pinocchio_interface.hpp"
#include "ctrl_component.hpp"
#include <iostream>

class KalmanFilterEstimator
{
public:
    KalmanFilterEstimator(shared_ptr<PinocchioInterface> pin_interface, double dt);
    void update(const LowState &low_state, const Vector4i &contact_flag, RobotState &robot_state);

private:
    void updateCovarianceMatrix(const Vector4i &contact_flag);
    void updateMeasurement(RobotState &robot_state);

    static constexpr int kFeetNum = 4;   // number of feet
    static constexpr int kFeetDim = 12;  // 3 * feet number
    static constexpr int kStateDim = 18; // [positon, velocity, foot position]
    static constexpr int kMeasDim = 28;  // [four feet rela position, four feet rela velocity, foot height]
    const double kLargeVariance_ = 1000;

    const Vector3d kGravity_{0, 0, -9.81};
    const Matrix3d kIdentity3_ = Matrix3d::Identity();
    const Vector4d kFeetHeight_ = Vector4d::Zero();

    shared_ptr<PinocchioInterface> pin_interface_;

    Eigen::Matrix<double, kStateDim, kStateDim> A_;                       // State matrix in x = Ax + Bu
    Eigen::Matrix<double, kStateDim, 3> B_;                               // Measurement matrix in z = H
    Eigen::Matrix<double, kMeasDim, kStateDim> H_;                        // Measurement matrix in z = H
    Eigen::Matrix<double, kStateDim, kMeasDim> H_T_;                      // Transpose of measurement matrix in z = H
    Eigen::Matrix<double, kStateDim, kStateDim> Q_;                       // State transition covariance
    Eigen::Matrix<double, kStateDim, kStateDim> Q_init_;                  // Initial value of state transition covariance
    Eigen::Matrix<double, kMeasDim, kMeasDim> R_;                         // Measurement covariance
    Eigen::Matrix<double, kMeasDim, kMeasDim> R_init_;                    // Initial value of measurement covariance
    Eigen::Matrix<double, kStateDim, kStateDim> P_;                       // Prediction covariance
    Eigen::Matrix<double, kStateDim, kStateDim> P_priori_;                // Priori prediction covariance
    Eigen::Matrix<double, kStateDim, 1> x_hat_;                           // [positon, velocity, foot position]
    Eigen::Matrix<double, kMeasDim, 1> z_;                                // [four feet rela position, four feet rela velocity, foot height]
    Eigen::Matrix<double, kMeasDim, kMeasDim> S_;                         // S = H*P*H.T + R
    Eigen::Matrix<double, kMeasDim, 1> S_z_;                              // S_z = S.inv() * (z - Hx)
    Eigen::Matrix<double, kMeasDim, kStateDim> S_H_;                      // S_H = _S.inv() * H
    Eigen::PartialPivLU<Eigen::Matrix<double, kMeasDim, kMeasDim>> S_lu_; // S.lu()

    double foot_radius_ = 0.03;
    double noise_processimu_position_ = 0.02;
    double noise_processimu_velocity_ = 0.02;
    double noise_process_foot_position_ = 0.002;
    double noise_meas_joint_position_ = 0.005;
    double noise_meas_joint_velocity_ = 0.1;
    double noise_meas_foot_height_ = 0.01;
};
