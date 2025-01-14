#include "kalman_filter_estimator.hpp"
#include <iostream>

KalmanFilterEstimator::KalmanFilterEstimator(LowState *lowState, PinocchioInterface *pin_interface)
    : _lowState(lowState), pin_interface_(pin_interface)
{
    xHat_.setZero(kStateDim);
    xHat_(2) = 0.42;
    ps_.setZero(kFootDim);
    vs_.setZero(kFootDim);
    A_.setIdentity(kStateDim, kStateDim);
    B_.setZero(kStateDim, 3);

    // 初始化测量矩阵
    H_.setZero();
    for (size_t i = 0; i < kFootNum; ++i)
    {
        H_.block<3, 3>(3 * i, 0) = I3;
        H_.block<3, 3>(3 * (kFootNum + i), 3) = I3;
        H_(2 * kFootDim + i, 6 + 3 * i + 2) = 1.0;
    }
    H_.block(0, 6, kFootDim, kFootDim) = -MatrixXd::Identity(kFootDim, kFootDim);
    std::cout << "H_\n"
              << H_ << std::endl;

    // 初始化过程噪声协方差矩阵
    Q_init_.setIdentity();
    Q_init_.block(0, 0, 3, 3) = (dt / 20.f) * Matrix3d::Identity();
    Q_init_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Matrix3d::Identity();
    Q_init_.block(6, 6, kFootDim, kFootDim) = dt * MatrixXd::Identity(kFootDim, kFootDim);
    Q_ = Q_init_;
    Q_.block<3, 3>(0, 0) *= imu_process_noise_position_;
    Q_.block<3, 3>(3, 3) *= imu_process_noise_velocity_;
    Q_.block<kFootDim, kFootDim>(6, 6) *= footProcessNoisePosition_;

    p_ = 100. * Q_init_;
    r_.setIdentity(kMeasDim, kMeasDim);
    feet_heights_.setZero(kFootNum);
}
void KalmanFilterEstimator::update(const VecInt4 &contact_flag)
{
    _posB = xHat_.segment<3>(0);
    _velB = xHat_.segment<3>(3);

    _qLeg = _lowState->getQLeg();
    _dqLeg = _lowState->getDqLeg();

    _quatB = _lowState->getQuaternion();
    _rotB = quat2RotMat(_quatB);
    _angVelB_B = _lowState->getGyro();
    _angVelB << _rotB * _angVelB_B;
    _accB_B = _lowState->getAccelerometer();
    _accB = _rotB * _accB_B;

    double dt = 0.001;
    A_.block(0, 3, 3, 3) = dt * Matrix3d::Identity();
    B_.block(0, 0, 3, 3) = 0.5 * dt * dt * Matrix3d::Identity();
    B_.block(3, 0, 3, 3) = dt * Matrix3d::Identity();

    size_t actuatedDofNum = 12;

    _posGen.setZero();
    _posGen.segment(0, 3) = _posB;
    _posGen.segment(3, 4) = _quatB; // Only set orientation, let position in origin.
    _posGen.segment(7, actuatedDofNum) = vec34ToVec12(_qLeg);

    _velGen.setZero();
    _velGen.segment(0, 3) = _rotB.transpose() * _velB;
    _velGen.segment(3, 3) = _angVelB_B; // Only set angular velocity, let linear velocity be zero
    _velGen.segment(6, actuatedDofNum) = vec34ToVec12(_dqLeg);

    pin_interface_->updateKinematics(_posGen, _velGen);
    const auto &feet_id = pin_interface_->feet_id();
    for (size_t i = 0; i < feet_id.size(); i++)
    {
        _posF.col(i) = pin_interface_->calcFootPosition(feet_id[i]);
        _velF.col(i) = pin_interface_->calcFootVelocity(feet_id[i]);
        _posF2B.col(i) = _posF.col(i) - _posB;
        _velF2B.col(i) = _velF.col(i) - _velB;
    }
    std::cout << "_posF2B\n"
              << _posF2B << std::endl;
    std::cout << "_velF2B\n"
              << _velF2B << std::endl;

    MatrixXd r = MatrixXd::Identity(kMeasDim, kMeasDim);
    r.block(0, 0, kFootDim, kFootDim) =
        r_.block(0, 0, kFootDim, kFootDim) * footSensorNoisePosition_;
    r.block(kFootDim, kFootDim, kFootDim, kFootDim) =
        r_.block(kFootDim, kFootDim, kFootDim, kFootDim) * footSensorNoiseVelocity_;
    r.block(2 * kFootDim, 2 * kFootDim, kFootNum, kFootNum) =
        r_.block(2 * kFootDim, 2 * kFootDim, kFootNum, kFootNum) * footHeightSensorNoise_;

    for (int i = 0; i < kFootNum; i++)
    {
        int i1 = 3 * i;

        int qIndex = 6 + i1;
        int rIndex1 = i1;
        int rIndex2 = kFootDim + i1;
        int rIndex3 = 2 * kFootDim + i;
        bool isContact = contact_flag[i];

        double high_suspect_number(1000000);
        Q_.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * Q_init_.block(qIndex, qIndex, 3, 3);
        r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
        r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
        r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

        ps_.segment(3 * i, 3) = -_posF2B.col(i);
        ps_.segment(3 * i, 3)[2] += foot_radius_;
        vs_.segment(3 * i, 3) = -_velF2B.col(i);
    }

    Vector3d g(0, 0, -9.81);
    Vector3d accel = _accB + g;

    VectorXd y(kMeasDim);
    y << ps_, vs_, feet_heights_;
    xHat_ = A_ * xHat_ + B_ * accel;
    MatrixXd at = A_.transpose();
    MatrixXd pm = A_ * p_ * at + Q_;
    MatrixXd cT = H_.transpose();
    MatrixXd yModel = H_ * xHat_;
    MatrixXd ey = y - yModel;
    MatrixXd s = H_ * pm * cT + r;

    VectorXd sEy = s.lu().solve(ey);
    xHat_ += pm * cT * sEy;

    MatrixXd sC = s.lu().solve(H_);
    p_ = (MatrixXd::Identity(kStateDim, kStateDim) - pm * cT * sC) * pm;

    MatrixXd pt = p_.transpose();
    p_ = (p_ + pt) / 2.0;

    std::cout << std::left << "Q_:     \n"
              << Q_ << std::endl;
    std::cout << std::left << "KLF Body Position:     \t"
              << _posB.transpose() << std::endl;
    std::cout << std::left << "KLF Body Velocity:     \t"
              << _velB.transpose() << std::endl;
}
