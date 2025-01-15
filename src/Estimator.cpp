#include "Estimator.h"
#include <iostream>

Estimator::Estimator(LowState *lowState, PinocchioInterface *pin_interface)
    : _lowState(lowState), pin_interface_(pin_interface)
{
}

void Estimator::update()
{
    // joint
    _qLeg = _lowState->getQLeg();
    _dqLeg = _lowState->getDqLeg();

    // body
    body_state_.pos << _lowState->supervisor.robotPos[0], _lowState->supervisor.robotPos[1], _lowState->supervisor.robotPos[2];
    body_state_.vel << _lowState->supervisor.robotVel[0], _lowState->supervisor.robotVel[1], _lowState->supervisor.robotVel[2];
    body_state_.quat = _lowState->getQuaternion();
    body_state_.rotmat = quat2RotMat(body_state_.quat);
    body_state_.angvel << body_state_.rotmat * _lowState->getGyro();
    body_state_.vel_B = body_state_.rotmat.transpose() * body_state_.vel;
    body_state_.angvel_B = body_state_.rotmat.transpose() * body_state_.angvel;
    
    // CoM
    _posGen << body_state_.pos,  body_state_.quat, vec34ToVec12(_qLeg), _qArm;
    _velGen << body_state_.vel_B, body_state_.angvel_B, vec34ToVec12(_dqLeg), _dqArm;
    pin_interface_->calcComState(_posGen, _velGen, _posCoM, _velCoM);

    // foot
    pin_interface_->updateKinematics(_posGen, _velGen);
    const auto &feet_id = pin_interface_->feet_id();
    for (size_t i = 0; i < feet_id.size(); i++)
    {
        _posF.col(i) = pin_interface_->calcFootPosition(feet_id[i]);
        _velF.col(i) = pin_interface_->calcFootVelocity(feet_id[i]);
        _posF2B.col(i) = _posF.col(i) - body_state_.pos;
        _velF2B.col(i) = _velF.col(i) - body_state_.vel;
    }

    // arm
    _qArm = _lowState->getQArm();
    _dqArm = _lowState->getDqArm();
    pin_interface_->calcGripperState(_posG, _velG, _quatG, _angVelG);

    // // control
    _workMode = WorkMode::ARM_JOINT;
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
              << body_state_.pos.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Current Body Velocity:     \t"
              << body_state_.vel.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "LKF Body Position:     \t" << std::fixed << std::setprecision(2)
    //           << x_.head(3).transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "LKF Body Velocity:     \t" << std::fixed << std::setprecision(2)
    //           << x_.tail(3).transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Position:  \t" << std::fixed << std::setprecision(2)
    //           << body_state_.pos.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Velocity:  \t" << std::fixed << std::setprecision(2)
    //           << body_state_.vel.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Arm Joint position:\t" << std::fixed << std::setprecision(2)
    //           << _qArm.transpose() << std::endl;
}
