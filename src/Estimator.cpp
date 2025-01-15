#include "Estimator.h"
#include <iostream>

Estimator::Estimator(LowState *lowState, PinocchioInterface *pin_interface)
    : _lowState(lowState), pin_interface_(pin_interface)
{
}


void Estimator::update(const Vec4 &phase, const Vector4i &contact, double T_sw, double T_st)
{
    // gait
    _phase = phase;
    _contact = contact;
    T_st_ = T_st;
    T_sw_ = T_sw;

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

    // // control
    // switch (_lowState->getUserCmd())
    // {
    // case UserCommand::A:
    //     _workMode = WorkMode::ARM_CARTESIAN_BODY;
    //     break;
    // case UserCommand::B:
    //     _workMode = WorkMode::ARM_JOINT;
    //     break;
    // case UserCommand::X:
    //     _workMode = WorkMode::ARM_FIXED_BODY;
    //     break;
    // case UserCommand::Y:
    //     _workMode = WorkMode::ARM_FIXED_WORLD;
    //     break;
    // }
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
