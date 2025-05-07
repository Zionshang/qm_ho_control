#include "cheat_estimator.hpp"

CheatEstimator::CheatEstimator(shared_ptr<PinocchioInterface> pin_interface)
    : pin_interface_(pin_interface) {}

void CheatEstimator::update(const webots::Node *cheat_node, const LowState &low_state, RobotState &robot_state)
{
    const double *cheat_body_pos = cheat_node->getPosition();
    const double *cheat_body_vel = cheat_node->getVelocity();

    // body
    auto &body_state = robot_state.body;
    for (int i = 0; i < 3; i++)
    {
        body_state.pos(i) = static_cast<double>(cheat_body_pos[i]);
        body_state.vel(i) = static_cast<double>(cheat_body_vel[i]);
    }

    body_state.quat = low_state.getQuaternion();
    RotMat rotmat_body_ = body_state.quat.toRotationMatrix();
    body_state.angvel = rotmat_body_ * low_state.getGyro();

    // joint
    auto &joint_state = robot_state.joint;
    joint_state.pos_leg = low_state.getLegJointPosition();
    joint_state.vel_leg = low_state.getLegJointVelocity();
    joint_state.pos_arm = low_state.getArmJointPosition();
    joint_state.vel_arm = low_state.getArmJointVelocity();

    // gen
    robot_state.pos_gen << body_state.pos,
        body_state.quat.coeffs(),
        mat34ToVec12(joint_state.pos_leg),
        joint_state.pos_arm;
    robot_state.vel_gen << rotmat_body_.transpose() * body_state.vel,
        rotmat_body_.transpose() * body_state.angvel,
        mat34ToVec12(joint_state.vel_leg),
        joint_state.vel_arm;

    // foot
    auto &foot_state = robot_state.foot;
    pin_interface_->updateKinematics(robot_state.pos_gen, robot_state.vel_gen);
    for (size_t i = 0; i < 4; i++)
    {
        foot_state.pos.col(i) = pin_interface_->getFootPosition(i);
        foot_state.vel.col(i) = pin_interface_->getFootVelocity(i);
        foot_state.pos_rel_body.col(i) = foot_state.pos.col(i) - robot_state.body.pos;
        foot_state.vel_rel_body.col(i) = foot_state.vel.col(i) - robot_state.body.vel;
    }

    // com
    pin_interface_->calcComState(robot_state.pos_gen, robot_state.vel_gen, robot_state.pos_com, robot_state.vel_com);

    // std::cout << "===== Robot State =====" << std::endl;

    // // Body state
    std::cout << "Body Position: " << robot_state.body.pos.transpose() << std::endl;
    std::cout << "Body Quaternion: " << robot_state.body.quat.coeffs().transpose() << std::endl;
    std::cout << "Body Velocity: " << robot_state.body.vel.transpose() << std::endl;
    std::cout << "Body Angular Velocity: " << robot_state.body.angvel.transpose() << std::endl;

    // Generalized State
    std::cout << "Generalized Position: " << robot_state.pos_gen.transpose() << std::endl;
    std::cout << "Generalized Velocity: " << robot_state.vel_gen.transpose() << std::endl;

    // Center of Mass references
    std::cout << "CoM Position: " << robot_state.pos_com.transpose() << std::endl;
    std::cout << "CoM Velocity: " << robot_state.vel_com.transpose() << std::endl;
}
