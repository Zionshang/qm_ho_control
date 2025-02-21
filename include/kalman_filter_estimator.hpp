#pragma once
#include "common/mathTypes.h"
#include "common/LowState.h"
#include "pinocchio_interface.hpp"
#include <iostream>

class KalmanFilterEstimator
{
public:
    KalmanFilterEstimator(LowState *lowState, PinocchioInterface *pin_interface, double dt);
    void update(const Vector4i &contact_flag);

    const Vector3d &pos_body() const { return pos_body_; }      // position body, expressed in world frame
    const Vector3d &vel_body() const { return vel_body_; }      // velocity body, expressed in world frame
    const Quaternion &getQuatB() const { return quat_body_; }   // quaternion of body frame relative to world frame
    const RotMat &getRotB() const { return rotmat_body_; }      // rotation matrix of body frame relative to world frame
    const Vector3d &getAngVelB() const { return angvel_body_; } // angular velocity of body, expressed in world frame

    const Vector3d &pos_com() const { return pos_com_; } // get position of CoM, expressed in world frame
    const Vector3d &vel_com() const { return vel_com_; } // get velocity of CoM, expressed in world frame

    const Matrix34d &pos_leg() const { return pos_leg_; }   // joint position of leg
    const Matrix34d &vel_leg() const { return vel_leg_; }   // joint velocity of leg
    const Matrix34d &pos_feet() const { return pos_feet_; } // position of four feet, expressed in world frame
    const Matrix34d &vel_feet() const { return vel_feet_; } // velocity of four feet, expressed in world frame

    double getCurrentTime() const { return low_state_->getCurrentTime(); }

private:
    void updateCovarianceMatrix(const Vector4i &contact_flag);
    void updateMeasurement();

    static constexpr int kFeetNum = 4;   // number of feet
    static constexpr int kFeetDim = 12;  // 3 * feet number
    static constexpr int kStateDim = 18; // [positon, velocity, foot position]
    static constexpr int kMeasDim = 28;  // [four feet rela position, four feet rela velocity, foot height]
    const double kLargeVariance_ = 1000;

    const Vector3d kGravity_{0, 0, -9.81};
    const Matrix3d kIdentity3_ = Matrix3d::Identity();
    const Vector4d kFeetHeight_ = Vector4d::Zero();

    LowState *low_state_;               // TODO: 改成智能指针
    PinocchioInterface *pin_interface_; // TODO: 改成智能指针

    Vector3d pos_body_;      // position body, expressed in world frame
    Vector3d vel_body_;      // velocity body, expressed in world frame
    RotMat rotmat_body_;     // rotation matrix of body frame relative to world frame
    Quaternion quat_body_;   // quaternion of body frame relative to world frame
    Vector3d angvel_body_;   // angular velocity of body, expressed in world frame
    Vector3d angvel_body_B_; // angular velocity of body, expressed in world frame

    Vector6d pos_arm_;  // joint position of arm
    Vector6d vel_arm_;  // joint velocity of arm
    Matrix34d pos_leg_; // joint position of leg
    Matrix34d vel_leg_; // joint velocity of leg

    VectorXd pos_gen_; // generalized position [pos_body, vel_body, pos_leg, pos_arm]
    VectorXd vel_gen_; // generalized velocity [vel_body_B, angvel_body_B, vel_leg, vel_arm]
    Vector3d pos_com_; // position of CoM, expressed in world frame
    Vector3d vel_com_; // position of CoM, expressed in world frame

    Matrix34d pos_feet_;         // position of four feet, expressed in world frame
    Matrix34d vel_feet_;         // velocity of four feet, expressed in world frame
    Matrix34d pos_feet_rel_body; // position feet relative to body, expressed in world frame
    Matrix34d vel_feet_rel_body; // velocity of feet relative to body, expressed in world frame

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

    double foot_radius_ = 0.02;
    double noise_process_imu_position_ = 0.02;
    double noise_process_imu_velocity_ = 0.02;
    double noise_process_foot_position_ = 0.002;
    double noise_meas_joint_position_ = 0.005;
    double noise_meas_joint_velocity_ = 0.1;
    double noise_meas_foot_height_ = 0.01;
};
