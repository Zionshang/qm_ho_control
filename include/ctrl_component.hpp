#pragma once
#include "common/types.hpp"
#include "common/enum_class.hpp"
#include "common/low_state.hpp"
#include "common/low_cmd.hpp"
#include "pinocchio_interface.hpp"

struct BodyState
{
    Vector3d pos;     // position of body, expressed in world frame
    Quaterniond quat; // quaternion of body relative to world frame
    Vector3d vel;     // velocity of body, expressed in world frame
    Vector3d angvel;  // angluar velocity of body, expressed in world frame

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
    VectorXd pos_arm;  // arm joint position
    VectorXd vel_arm;  // arm joint velocity

    JointState(int arm_nq)
    {
        pos_leg.setZero();
        vel_leg.setZero();
        pos_arm.setZero(arm_nq);
        vel_arm.setZero(arm_nq);
        pos_arm << 0, -1.57, 2.88, 0.26, 0;
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
        : joint(nv - 18)
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
    VectorXd arm_joint_pos;
    ControllerType ctrl_type;

    UserCommand(int arm_nq)
    {
        vel_body_B.setZero();
        angvel_body_B.setZero();
        gait_name = GaitName::STANCE;
        ctrl_type = ControllerType::TORQUE_CONTROLLER;
        arm_joint_pos.setZero(arm_nq);
        arm_joint_pos << 0, -1.57, 2.88, 0.26, 0;
    }
};

struct CtrlComponent
{
    RobotState robot_state;
    UserCommand user_cmd;
    RobotState target_robot_state;

    LowState low_state;
    LowCmd low_cmd;

    CtrlComponent(shared_ptr<PinocchioInterface> pin_interface)
        : robot_state(pin_interface->nq(), pin_interface->nv()),
          target_robot_state(pin_interface->nq(), pin_interface->nv()),
          user_cmd(pin_interface->nv() - 18)
    {
        target_robot_state.body.pos << 0.0, 0.0, 0.57; // todo: 是否必要存在？
        target_robot_state.pos_com << 0, 0, 0.527;
    }
};
