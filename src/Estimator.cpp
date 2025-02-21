#include "Estimator.h"
#include <iostream>

Estimator::Estimator(LowState *lowState, PinocchioInterface *pin_interface)
    : _lowState(lowState), pin_interface_(pin_interface) {}

void Estimator::update(RobotState &robot_state)
{
    // joint
    auto &joint_state = robot_state.joint;
    joint_state.pos_leg = _lowState->getQLeg();
    joint_state.vel_leg = _lowState->getDqLeg();
    joint_state.pos_arm = _lowState->getQArm();
    joint_state.vel_arm = _lowState->getDqArm();

    // body
    auto &body_state = robot_state.body;
    body_state.pos << _lowState->supervisor.robotPos[0], _lowState->supervisor.robotPos[1], _lowState->supervisor.robotPos[2];
    body_state.vel << _lowState->supervisor.robotVel[0], _lowState->supervisor.robotVel[1], _lowState->supervisor.robotVel[2];
    body_state.quat = _lowState->getQuaternion();
    body_state.rotmat = quat2RotMat(body_state.quat);
    body_state.angvel << body_state.rotmat * _lowState->getGyro();
    RotMat R_T = body_state.rotmat.transpose();

    // CoM
    auto &pos_gen = robot_state.pos_gen;
    auto &vel_gen = robot_state.vel_gen;
    auto &pos_com = robot_state.pos_com;
    auto &vel_com = robot_state.vel_com;
    pos_gen << body_state.pos, body_state.quat, vec34ToVec12(joint_state.pos_leg), joint_state.pos_arm;
    vel_gen << R_T * body_state.vel, R_T * body_state.angvel, vec34ToVec12(joint_state.vel_leg), joint_state.vel_arm;
    pin_interface_->calcComState(pos_gen, vel_gen, pos_com, vel_com);

    // foot
    auto &foot_state = robot_state.foot;
    pin_interface_->updateKinematics(pos_gen, vel_gen);
    const auto &feet_id = pin_interface_->feet_id();
    for (size_t i = 0; i < feet_id.size(); i++)
    {
        foot_state.pos.col(i) = pin_interface_->calcFootPosition(feet_id[i]);
        foot_state.vel.col(i) = pin_interface_->calcFootVelocity(feet_id[i]);
        foot_state.pos_rel_body.col(i) = foot_state.pos.col(i) - body_state.pos;
        foot_state.vel_rel_body.col(i) = foot_state.vel.col(i) - body_state.vel;
    }

    // arm
    pin_interface_->calcGripperState(_posG, _velG, _quatG, _angVelG);

}

void Estimator::printState()
{
    // std::cout << std::setw(12) << std::left << "Current Body Position:     \t"
    //           << body_state_.pos.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Body Velocity:     \t"
    //           << body_state_.vel.transpose() << std::endl;
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
