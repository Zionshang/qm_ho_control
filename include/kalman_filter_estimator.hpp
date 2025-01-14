#pragma once
#include "common/mathTypes.h"
#include "common/LowState.h"
#include "pinocchio_interface.hpp"

class KalmanFilterEstimator
{
public:
    KalmanFilterEstimator(LowState *lowState, PinocchioInterface *pin_interface);
    void update(const VecInt4 &contact_flag);

    // body
    Vec3 getPosB() const { return _posB; }       // get position of BODY, expressed in WORLD frame
    Vec3 getVelB() const { return _velB; }       // get velocity of BODY, expressed in WORLD frame
    Quat getQuatB() const { return _quatB; }     // get quaternion of BODY relative to WORLD
    RotMat getRotB() const { return _rotB; }     // get rotation matrix of BODY relative to WORLD
    Vec3 getAngVelB() const { return _angVelB; } // get angular velocity of BODY, expressed in WORLD frame

    // CoM
    Vec3 getPosCoM() const { return _posCoM; } // get position of CoM, expressed in WORLD frame
    Vec3 getVelCoM() const { return _velCoM; } // get velocity of CoM, expressed in WORLD frame

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
    const double dt = 0.001; // timestep

    const Vector3d gravity_{0, 0, -9.81};

    LowState *_lowState;
    PinocchioInterface *pin_interface_;

    // body
    Vec3 _posB, _velB; // position and velocity BODY, expressed in WORLD frame
    RotMat _rotB;      // rotation matrix of BODY relative to WORLD
    Quat _quatB;       // quaternion of BODY relative to WORLD
    Vec3 _angVelB;     // angular velocity of BODY, expressed in WORLD frame
    Vec3 _angVelB_B;   // angular velocity of BODY, expressed in WORLD frame
    Vec3 _accB_B, _accB;

    // CoM
    VecNq _posGen;
    VecNv _velGen;
    Vec3 _posCoM, _velCoM; // position and velocity CoM, expressed in WORLD frame

    // leg & foot
    Vec34 _qLeg, _dqLeg;    // joint position and velocity of leg
    Vec34 _posF, _velF;     // position and velocity of FOOT, expressed in WORLD frame
    Vec34 _posF2B, _velF2B; // position and velocity of FOOT relative to BODY, expressed in WORLD frame

    VectorXd feet_heights_;
    static constexpr int kFootNum = 4;
    static constexpr int kFootDim = 12;
    static constexpr int kStateDim = 18; // [positon, velocity, foot position]
    static constexpr int kMeasDim = 28;  // [four feet rela position, four feet rela velocity, foot height]

    MatrixXd A_;                                         // State matrix in x = Ax + Bu
    MatrixXd B_;                                         // State matrix in x = Ax + Bu
    Eigen::Matrix<double, kMeasDim, kStateDim> H_;       // Measurement matrix in z = H
    Eigen::Matrix<double, kStateDim, kStateDim> Q_init_; // Initial value of state transition covariance
    Eigen::Matrix<double, kStateDim, kStateDim> Q_;      // State transition covariance
    Eigen::Matrix<double, kStateDim, kStateDim> p_;      // Prediction covariance
    MatrixXd r_;                                         // Measurement covariance
    VectorXd xHat_;                                      // [position, velocity]
    VectorXd ps_;                                        // 测量位置
    VectorXd vs_;                                        // 测量速度

    Matrix3d I3 = Matrix3d::Identity();

    double foot_radius_ = 0.02;
    double imu_process_noise_position_ = 0.02;
    double imu_process_noise_velocity_ = 0.02;
    double footProcessNoisePosition_ = 0.002;
    double footSensorNoisePosition_ = 0.005;
    double footSensorNoiseVelocity_ = 0.1;
    double footHeightSensorNoise_ = 0.01;
};
