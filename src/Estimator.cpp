#include "Estimator.h"
#include <iostream>

Estimator::Estimator(GaitName gaitName, LowState *lowState, PinocchioInterface *pin_interface)
    : _lowState(lowState), pin_interface_(pin_interface)
{
    _gaitSche = new GaitSchedule(gaitName);

    // 初始化状态向量
    x_.setZero();
    x_(2) = 0.42;

    // 初始化协方差矩阵
    P_.setIdentity();

    // 初始化状态转移矩阵
    A_.setIdentity();
    A_.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;

    // 初始化测量矩阵
    H_.setZero();
    H_.block<3, 3>(0, 3) = Matrix3d::Identity();

    // 初始化过程噪声
    Q_.setIdentity() * 1e-4;
    Q_.block<3, 3>(0, 0) *= 1e-6; // 位置过程噪声
    Q_.block<3, 3>(3, 3) *= 1e-2; // 速度过程噪声

    // 初始化测量噪声
    R_.setIdentity();
    R_.block<3, 3>(0, 0) *= 1e-2; // 速度测量噪声

    std::cout << "A\n"
              << A_ << std::endl;
    std::cout << "H\n"
              << H_ << std::endl;
    // Q_ = 0.0001 * Q_.setIdentity(9, 9);

    // Q_.block(0, 0, 3, 3) = (0.001 / 20) * Q_.block(0, 0, 3, 3).setIdentity();       // 位置噪声
    // Q_.block(6, 6, 3, 3) = (0.001 / 20 * 9.8) * Q_.block(3, 3, 3, 3).setIdentity(); // 速度噪声

    // R_.setIdentity(6, 6); // 观测值噪声矩阵
    // R_ = 2.5 * R_.setIdentity(6, 6);
    // R_(5, 5) = 0.1;
    // R_.block(0, 0, 3, 3) = 0.001 * Eigen::Matrix3d::Identity();
}

Estimator::~Estimator()
{
    delete _gaitSche;
}

void Estimator::setAllState()
{
    // gait
    _gaitSche->run(_lowState->currentTime);
    _phase = _gaitSche->getPhase();
    _contact = _gaitSche->getContact();

    // joint
    _qLeg = _lowState->getQLeg();
    _dqLeg = _lowState->getDqLeg();

    // body
    _posB << _lowState->supervisor.robotPos[0], _lowState->supervisor.robotPos[1], _lowState->supervisor.robotPos[2];
    _velB << _lowState->supervisor.robotVel[0], _lowState->supervisor.robotVel[1], _lowState->supervisor.robotVel[2];
    _quatB = _lowState->getQuaternion();
    _rotB = quat2RotMat(_quatB);
    _angVelB << _rotB * _lowState->getGyro();

    // CoM
    _posGen << _posB, _quatB, vec34ToVec12(_qLeg), _qArm;
    _velGen << _rotB.transpose() * _velB, _rotB.transpose() * _angVelB, vec34ToVec12(_dqLeg), _dqArm;
    pin_interface_->calcComState(_posGen, _velGen, _posCoM, _velCoM);

    // foot
    pin_interface_->updateKinematics(_posGen, _velGen);
    const auto &feet_id = pin_interface_->feet_id();
    for (size_t i = 0; i < feet_id.size(); i++)
    {
        _posF.col(i) = pin_interface_->calcFootPosition(feet_id[i]);
        _velF.col(i) = pin_interface_->calcFootVelocity(feet_id[i]);
        _posF2B.col(i) = _posF.col(i) - _posB;
        _velF2B.col(i) = _velF.col(i) - _velB;
    }

    // arm
    _qArm = _lowState->getQArm();
    _dqArm = _lowState->getDqArm();
    pin_interface_->calcGripperState(_posG, _velG, _quatG, _angVelG);

    // control
    switch (_lowState->getUserCmd())
    {
    case UserCommand::A:
        _workMode = WorkMode::ARM_CARTESIAN_BODY;
        break;
    case UserCommand::B:
        _workMode = WorkMode::ARM_JOINT;
        break;
    case UserCommand::X:
        _workMode = WorkMode::ARM_FIXED_BODY;
        break;
    case UserCommand::Y:
        _workMode = WorkMode::ARM_FIXED_WORLD;
        break;
    }

    // Vector3d acc = _lowState->getAccelerometer();                          // world frame
    // Vector3d velocity = -_velF * _contact.cast<double>() / _contact.sum(); // world frame
    // if (_contact.sum() == 0)
    //     velocity = Eigen::Vector3d::Zero();

    // updateStateEquation(acc);
    // updateMeasurementEquation(velocity);
    // Eigen::Vector3d x_j = -_posF2B * _contact.cast<double>() / _contact.sum();
    // x_(2) = x_j(2);
    // if (_contact.sum() == 0)
    //     x_(2) = 0;
}

void Estimator::updateStateEquation(const Eigen::Vector3d &acc)
{
    // 更新状态
    x_.segment(0, 3) += x_.segment(3, 3) * dt + 0.5 * (acc + gravity_) * dt * dt;
    x_.segment(3, 3) += (acc + gravity_) * dt;

    // 更新协方差
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void Estimator::updateMeasurementEquation(const Vector3d &measured_velocity)
{
    z_ << measured_velocity;

    // 计算卡尔曼增益
    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

    // 计算创新
    Eigen::Matrix<double, kMeasDim, 1> innovation = z_ - H_ * x_;

    // 更新状态
    x_ += K_ * innovation;

    // 更新协方差
    Eigen::Matrix<double, kStateDim, kStateDim> I = Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    P_ = (I - K_ * H_) * P_;
}

void Estimator::printState()
{
    std::cout << std::setw(12) << std::left << "Current Body Position:     \t" 
              << _posB.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Current Body Velocity:     \t" 
              << _velB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "LKF Body Position:     \t" << std::fixed << std::setprecision(2)
    //           << x_.head(3).transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "LKF Body Velocity:     \t" << std::fixed << std::setprecision(2)
    //           << x_.tail(3).transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Position:  \t" << std::fixed << std::setprecision(2)
    //           << _posB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Velocity:  \t" << std::fixed << std::setprecision(2)
    //           << _velB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Arm Joint position:\t" << std::fixed << std::setprecision(2)
    //           << _qArm.transpose() << std::endl;
}
