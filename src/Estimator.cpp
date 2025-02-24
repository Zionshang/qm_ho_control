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
    RotMat R_body = body_state.quat.toRotationMatrix();
    body_state.angvel << R_body * _lowState->getGyro();
    RotMat R_T = R_body.transpose();

    // CoM
    auto &pos_gen = robot_state.pos_gen;
    auto &vel_gen = robot_state.vel_gen;
    auto &pos_com = robot_state.pos_com;
    auto &vel_com = robot_state.vel_com;
    pos_gen << body_state.pos, body_state.quat.coeffs(), vec34ToVec12(joint_state.pos_leg), joint_state.pos_arm;
    vel_gen << R_T * body_state.vel, R_T * body_state.angvel, vec34ToVec12(joint_state.vel_leg), joint_state.vel_arm;
    pin_interface_->calcComState(pos_gen, vel_gen, pos_com, vel_com);

    // foot
    auto &foot_state = robot_state.foot;
    pin_interface_->updateKinematics(pos_gen, vel_gen);
    for (size_t i = 0; i < 4; i++)
    {
        foot_state.pos.col(i) = pin_interface_->getFootPosition(i);
        foot_state.vel.col(i) = pin_interface_->getFootVelocity(i);
        foot_state.pos_rel_body.col(i) = foot_state.pos.col(i) - body_state.pos;
        foot_state.vel_rel_body.col(i) = foot_state.vel.col(i) - body_state.vel;
    }

    // std::cout << "===== Robot State =====" << std::endl;

    // // // Body state
    // std::cout << "Body Position: " << robot_state.body.pos.transpose() << std::endl;
    // std::cout << "Body Quaternion: " << robot_state.body.quat.coeffs().transpose() << std::endl;
    // std::cout << "Body Velocity: " << robot_state.body.vel.transpose() << std::endl;
    // std::cout << "Body Angular Velocity: " << robot_state.body.angvel.transpose() << std::endl;
    // std::cout << "Body Rotation Matrix:" << std::endl
    //           << robot_state.body.rotmat << std::endl;

    // // Foot state
    // std::cout << "Foot Position: " << robot_state.foot.pos.transpose() << std::endl;
    // std::cout << "Foot Velocity: " << robot_state.foot.vel.transpose() << std::endl;

    // // Joint state
    // std::cout << "Leg Joint Position: " << robot_state.joint.pos_leg << std::endl;
    // std::cout << "Leg Joint Velocity: " << robot_state.joint.vel_leg << std::endl;
    // std::cout << "Arm Joint Velocity: " << robot_state.joint.vel_arm.transpose() << std::endl;
    // std::cout << "Arm Joint Velocity: " << robot_state.joint.vel_arm.transpose() << std::endl;

    // // Generalized State
    // std::cout << "Generalized Position: " << robot_state.pos_gen.transpose() << std::endl;
    // std::cout << "Generalized Velocity: " << robot_state.vel_gen.transpose() << std::endl;

    // // Center of Mass references
    // std::cout << "CoM Position Ref: " << robot_state.pos_com.transpose() << std::endl;
    // std::cout << "CoM Velocity Ref: " << robot_state.vel_com.transpose() << std::endl;
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
