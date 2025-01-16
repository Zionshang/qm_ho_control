#pragma once
#include <common/mathTypes.h>
#include "pinocchio_interface.hpp"

struct BodyState
{
    Vector3d pos;    // position of body, expressed in world frame
    Quaternion quat; // quaternion of body relative to world frame
    RotMat rotmat;   // rotation matrix of body relative to world frame
    Vector3d vel;    // velocity of body, expressed in world frame
    Vector3d angvel; // angluar velocity of body, expressed in world frame
};

struct FootState
{
    Matrix34d pos; // position of foot, expressed in world frame
    Matrix34d vel; // velocity of foot, expressed in world frame
};

struct JointState
{
    VectorXd pos_leg; // leg joint position
    VectorXd vel_leg; // leg joint velocity
    VectorXd pos_arm; // arm joint position
    VectorXd vel_arm; // arm joint velocity
};

struct RobotState
{
    BodyState body;   // body state
    FootState foot;   // four feet state
    JointState joint; // joint state
    VectorXd pos_gen; // general position
    VectorXd vel_gen; // general velocity
    Vector3d pos_com; // position of center of mass, expressed in world frame
    Vector3d vel_com; // velocity of center of mass, expressed in world frame
};

class CtrlComponent
{
public:
    CtrlComponent(PinocchioInterface *pin_interface)
    {
        robot_state_.pos_gen.setZero(pin_interface->nq());
        robot_state_.vel_gen.setZero(pin_interface->nv());
    }

    const RobotState &robot_state() const { return robot_state_; }
    RobotState &mutable_robot_state() { return robot_state_; }

private:
    RobotState robot_state_;
};
