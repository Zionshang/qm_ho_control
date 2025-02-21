#include "Planner.h"

Planner::Planner(Estimator *est, LowState *lowState, WholeBodyDynamics *wbDyn)
    : _est(est), _lowState(lowState), _wbDyn(wbDyn)
{
    _gaitGen = new FootPlanner();

    dt_ = _est->getTimeStep();
    _velBMax << 0.5, 0.5, 0.3;    // x,y,z direction
    _angVelBMax << 0.5, 0.5, 0.5; // x,y,z direction
    _velBMin = -_velBMax;
    _angVelBMin = -_angVelBMax;
    _velBCmd.setZero();
    _angVelBCmd.setZero();

    _velGMax << 0.5, 0.5, 0.5;    // x,y,z direction
    _angVelGMax << 0.5, 0.5, 0.5; // x,y,z direction
    _velGMin = -_velGMax;
    _angVelGMin = -_angVelGMax;
    _velGCmd.setZero();
    _angVelGCmd.setZero();

    // clang-format off
    _qAJMin  << -2.61,    0, -2.87, -1.51, -1.34, -2.79;
    _qAJMax  <<  2.61, 2.96,     0,  1.51,  1.34,  2.79;
    _dqAJMax <<   0.5,  0.5,   0.5,   0.5,   0.5,   0.5;
    // clang-format on
    _dqAJMax = 3 * _dqAJMax.eval();
    _dqAJMin = -_dqAJMax;
    _dqAJCmd.setZero();
}

Planner::~Planner()
{
    delete _gaitGen;
};

void Planner::update(const UserCommand &user_cmd, const RobotState &robot_state,
                     const GaitState &gait_state, RobotState &robot_state_ref)
{

    bodyPlan(user_cmd, robot_state_ref.body);
    armJointPlan(robot_state_ref.joint);
    comPlan(robot_state_ref, robot_state_ref.pos_com, robot_state_ref.vel_com);
    _gaitGen->update(gait_state, robot_state.body, robot_state.foot, robot_state_ref.body, robot_state_ref.foot); // foot trajectory

    // std::cout << "===== Robot State Reference =====" << std::endl;

    // // Body state
    // std::cout << "Body Position: " << robot_state_ref.body.pos.transpose() << std::endl;
    // std::cout << "Body Quaternion: " << robot_state_ref.body.quat.transpose() << std::endl;
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
    body_state_ref.rotmat = quat2RotMat(body_state_ref.quat);

    // velocity transformation from BODY to WORLD
    body_state_ref.vel = body_state_ref.rotmat * user_cmd.vel_body_B;
    body_state_ref.angvel = body_state_ref.rotmat * user_cmd.angvel_body_B;

    // position integration
    body_state_ref.pos += body_state_ref.vel * dt_;
    body_state_ref.quat += quatDeriv(body_state_ref.quat, user_cmd.angvel_body_B) * dt_;
    body_state_ref.quat /= body_state_ref.quat.norm();
}

// todo: 是否有存在的必要？
void Planner::comPlan(RobotState &robot_state_ref, Vector3d &pos_com_ref, Vector3d &vel_com_ref)
{
    robot_state_ref.pos_gen << robot_state_ref.body.pos,
        robot_state_ref.body.quat,
        VectorXd::Zero(12),
        robot_state_ref.joint.pos_arm;

    robot_state_ref.vel_gen << robot_state_ref.body.rotmat.transpose() * robot_state_ref.body.vel,
        robot_state_ref.body.rotmat.transpose() * robot_state_ref.body.angvel,
        VectorXd::Zero(12, 1),
        robot_state_ref.joint.vel_arm;

    _wbDyn->setCoMPosVel(robot_state_ref.pos_gen, robot_state_ref.vel_gen, pos_com_ref, vel_com_ref);
}

void Planner::armJointPlan(JointState &joint_state_ref)
{
    joint_state_ref.pos_arm.setZero();
    joint_state_ref.vel_arm.setZero();
}