#pragma once
#include "common/mathTypes.h"
#include "common/LowState.h"
#include "pinocchio_interface.hpp"
#include <iostream>

class KalmanFilterEstimator
{
public:
    KalmanFilterEstimator(LowState *lowState, PinocchioInterface *pin_interface);
    void update(const Vector4i &contact_flag);

    // body
    Vec3 getPosB() const { return _posB; }       // get position of BODY, expressed in WORLD frame
    Vec3 getVelB() const { return _velB; }       // get velocity of BODY, expressed in WORLD frame
    Quat getQuatB() const { return _quatB; }     // get quaternion of BODY relative to WORLD
    RotMat getRotB() const { return _rotB; }     // get rotation matrix of BODY relative to WORLD
    Vec3 getAngVelB() const { return _angVelB; } // get angular velocity of BODY, expressed in WORLD frame

    // // CoM
    // Vec3 getPosCoM() const { return _posCoM; } // get position of CoM, expressed in WORLD frame
    // Vec3 getVelCoM() const { return _velCoM; } // get velocity of CoM, expressed in WORLD frame

    // foot
    Vec34 getQLeg() const { return _qLeg; }            // get joint position of four legs
    Vec34 getDqLeg() const { return _dqLeg; }          // get joint velocity of four legs
    Vec3 getPosF(int i) const { return _posF.col(i); } // get position of id foot, expressed in WORLD frame
    Vec3 getVelF(int i) const { return _velF.col(i); } // get velocity of id foot, expressed in WORLD frame
    Vec34 getPosF() const { return _posF; }            // get position of four feet, expressed in WORLD frame
    Vec34 getVelF() const { return _velF; }            // get velocity of four feet, expressed in WORLD frame

    double getTimeStep() const { return _lowState->getTimeStep(); }
    double getCurrentTime() const { return _lowState->getCurrentTime(); }
    void printState();

private:
    void updateCovarianceMatrix(const Vector4i &contact_flag);
    void updateMeasurement();

    static constexpr int kFeetNum = 4;   // number of feet
    static constexpr int kFeetDim = 12;  // 3 * feet number
    static constexpr int kStateDim = 18; // [positon, velocity, foot position]
    static constexpr int kMeasDim = 28;  // [four feet rela position, four feet rela velocity, foot height]

    const double kTimeStep_ = 0.001; // timestep, second
    const double kLargeVariance_ = 1000;

    const Vector3d kGravity_{0, 0, -9.81};
    const Matrix3d kIdentity3_ = Matrix3d::Identity();
    const Vector4d kFeetHeight_ = Vector4d::Zero();

    LowState *_lowState;
    PinocchioInterface *pin_interface_;

    Vec3 _posB, _velB; // position and velocity BODY, expressed in WORLD frame
    RotMat _rotB;      // rotation matrix of BODY relative to WORLD
    Quat _quatB;       // quaternion of BODY relative to WORLD
    Vec3 _angVelB;     // angular velocity of BODY, expressed in WORLD frame
    Vec3 _angVelB_B;   // angular velocity of BODY, expressed in WORLD frame
    Vec3 _accB;

    Vector6d pos_arm_; // joint position of arm
    Vector6d vel_arm_; // joint velocity of arm

    VectorXd pos_gen_; // generalized position [pos_body, vel_body, pos_leg, pos_arm]
    VectorXd vel_gen_; // generalized velocity [vel_body_B, angvel_body_B, vel_leg, vel_arm]
    Vector3d pos_com_; // position of CoM, expressed in WORLD frame
    Vector3d vel_com;  // position of CoM, expressed in WORLD frame

    // leg & foot
    Vec34 _qLeg, _dqLeg;    // joint position and velocity of leg
    Vec34 _posF, _velF;     // position and velocity of FOOT, expressed in WORLD frame
    Vec34 _posF2B, _velF2B; // position and velocity of FOOT relative to BODY, expressed in WORLD frame

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
