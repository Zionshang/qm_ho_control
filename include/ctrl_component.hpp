#pragma once
#include "common/math_types.hpp"
#include "common/enum_class.hpp"
#include "pinocchio_interface.hpp"

struct BodyState
{
    Vector3d pos;    // position of body, expressed in world frame
    Quaterniond quat; // quaternion of body relative to world frame
    Vector3d vel;    // velocity of body, expressed in world frame
    Vector3d angvel; // angluar velocity of body, expressed in world frame

    BodyState()
    {
        pos.setZero();
        quat.setIdentity();
        vel.setZero();
        angvel.setZero();
    }
};

struct FootState
{
    Matrix34d pos;          // position of foot, expressed in world frame
    Matrix34d vel;          // velocity of foot, expressed in world frame
    Matrix34d pos_rel_body; // position of foot relative to body, expressed in world frame
    Matrix34d vel_rel_body; // velocity of foot, expressed in world frame

    FootState()
    {
        pos.setZero();
        vel.setZero();
        pos_rel_body.setZero();
        vel_rel_body.setZero();
    }
};

struct JointState
{
    Matrix34d pos_leg; // leg joint position
    Matrix34d vel_leg; // leg joint velocity
    Vector6d pos_arm;  // arm joint position
    Vector6d vel_arm;  // arm joint velocity

    JointState()
    {
        pos_leg.setZero();
        vel_leg.setZero();
        pos_arm.setZero();
        vel_arm.setZero();
    }
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

    RobotState(int nq, int nv)
    {
        pos_gen.setZero(nq);
        vel_gen.setZero(nv);
        pos_com.setZero();
        vel_com.setZero();
    }
};

struct UserCommand
{
    Vector3d vel_body_B;
    Vector3d angvel_body_B;
    GaitName gait_name;

    UserCommand()
    {
        vel_body_B.setZero();
        angvel_body_B.setZero();
        gait_name = GaitName::STANCE;
    }
};

class CtrlComponent
{
public:
    CtrlComponent(PinocchioInterface *pin_interface)
        : robot_state_(pin_interface->nq(), pin_interface->nv()),
          target_robot_state_(pin_interface->nq(), pin_interface->nv())
    {
        target_robot_state_.body.pos << 0.0, 0.0, 0.42;
        target_robot_state_.pos_com << 0.0317053, 0.00152422, 0.440351;
    }

    const RobotState &robot_state() const { return robot_state_; }
    RobotState &mutable_robot_state() { return robot_state_; }

    const UserCommand &user_cmd() const { return user_cmd_; }
    UserCommand &mutable_user_cmd() { return user_cmd_; }

    const RobotState &target_robot_state() const { return target_robot_state_; }
    RobotState &mutable_target_robot_state() { return target_robot_state_; }

private:
    RobotState robot_state_;
    UserCommand user_cmd_;
    RobotState target_robot_state_;
};
