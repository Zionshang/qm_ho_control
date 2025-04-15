#include "planner.hpp"

Planner::Planner(shared_ptr<PinocchioInterface> pin_interface, double timestep)
    : pin_interface_(pin_interface), dt_(timestep)
{
    foot_planner_ = std::make_shared<FootPlanner>(pin_interface);
}

void Planner::update(const UserCommand &user_cmd, const RobotState &robot_state,
                     const GaitState &gait_state, RobotState &robot_state_ref)
{

    bodyPlan(user_cmd, robot_state_ref.body);
    armJointPlan(user_cmd, robot_state_ref.joint);
    comPlan(robot_state_ref, robot_state_ref.pos_com, robot_state_ref.vel_com);
    foot_planner_->update(gait_state, robot_state.body, robot_state.foot,
                          robot_state_ref.body, robot_state_ref.foot);

    // std::cout << "===== Robot State Reference =====" << std::endl;

    // // // Body state
    // std::cout << "Body Position: " << robot_state_ref.body.pos.transpose() << std::endl;
    // std::cout << "Body Quaternion: " << robot_state_ref.body.quat.coeffs().transpose() << std::endl;
    // std::cout << "Body Velocity: " << robot_state_ref.body.vel.transpose() << std::endl;
    // std::cout << "Body Angular Velocity: " << robot_state_ref.body.angvel.transpose() << std::endl;
    // std::cout << "Body Rotation Matrix:" << std::endl
    //           << robot_state_ref.body.rotmat << std::endl;

    // // Foot state
    // std::cout << "Foot Position: " << robot_state_ref.foot.pos.transpose() << std::endl;
    // std::cout << "Foot Velocity: " << robot_state_ref.foot.vel.transpose() << std::endl;

    // // Joint state
    // std::cout << "Leg Joint Position: " << robot_state_ref.joint.pos_leg << std::endl;
    // std::cout << "Leg Joint Velocity: " << robot_state_ref.joint.vel_leg << std::endl;
    // std::cout << "Arm Joint Velocity: " << robot_state_ref.joint.vel_arm.transpose() << std::endl;
    // std::cout << "Arm Joint Velocity: " << robot_state_ref.joint.vel_arm.transpose() << std::endl;

    // // Generalized State
    // std::cout << "Generalized Position: " << robot_state_ref.pos_gen.transpose() << std::endl;
    // std::cout << "Generalized Velocity: " << robot_state_ref.vel_gen.transpose() << std::endl;

    // // Center of Mass references
    // std::cout << "CoM Position Ref: " << robot_state_ref.pos_com.transpose() << std::endl;
    // std::cout << "CoM Velocity Ref: " << robot_state_ref.vel_com.transpose() << std::endl;

    // std::cout << "=================================" << std::endl;
}

void Planner::bodyPlan(const UserCommand &user_cmd, BodyState &body_state_ref)
{
    RotMat R_body = body_state_ref.quat.toRotationMatrix();

    // velocity transformation from BODY to WORLD
    body_state_ref.vel = R_body * user_cmd.vel_body_B;
    body_state_ref.angvel = R_body * user_cmd.angvel_body_B;

    // position integration
    body_state_ref.pos += body_state_ref.vel * dt_;
    body_state_ref.quat *= pinocchio::quaternion::exp3(user_cmd.angvel_body_B * dt_);
}

// todo: 是否有存在的必要？
void Planner::comPlan(RobotState &robot_state_ref, Vector3d &pos_com_ref, Vector3d &vel_com_ref)
{
    RotMat R_body_T = robot_state_ref.body.quat.toRotationMatrix().transpose();

    robot_state_ref.pos_gen << robot_state_ref.body.pos,
        robot_state_ref.body.quat.coeffs(),
        VectorXd::Zero(12),
        robot_state_ref.joint.pos_arm;

    robot_state_ref.vel_gen << R_body_T * robot_state_ref.body.vel,
        R_body_T * robot_state_ref.body.angvel,
        VectorXd::Zero(12, 1),
        robot_state_ref.joint.vel_arm;

    pin_interface_->calcComState(robot_state_ref.pos_gen, robot_state_ref.vel_gen, pos_com_ref, vel_com_ref);
}

void Planner::armJointPlan(const UserCommand &user_cmd, JointState &joint_state_ref)
{
    joint_state_ref.pos_arm = user_cmd.arm_joint_pos;
    joint_state_ref.vel_arm.setZero();
}