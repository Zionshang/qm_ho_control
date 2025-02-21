#pragma once

#include "common/LowState.h"
#include "foot_planner.hpp"

class Planner
{
public:
    Planner(Estimator *est, LowState *lowState, WholeBodyDynamics *wbDyn);
    ~Planner();
    void update(const UserCommand &user_cmd, const RobotState &robot_state,
                const GaitState &gait_state, RobotState &robot_state_ref);

private:
    void bodyPlan(const UserCommand &user_cmd, BodyState &body_state_ref);
    void armJointPlan(JointState &joint_state_ref);
    void comPlan(RobotState &robot_state_ref, Vector3d &pos_com_ref, Vector3d &vel_com_ref);

    FootPlanner *_gaitGen;
    Estimator *_est;
    LowState *_lowState;
    WholeBodyDynamics *_wbDyn;

    double dt_;
    RotMat _rotB_d; // desired rotation matrix of BODY reletive to GLOBAL

    // body
    Vector3d _velBMax, _angVelBMax; // the maximum of body linear velocity and body angular velocity
    Vector3d _velBMin, _angVelBMin; // the minimum of body linear velocity and body angular velocity
    Vector3d _velBCmd, _angVelBCmd; // expressed in BODY frame

    // gripper
    Vector3d _velGMax, _angVelGMax; // the maximum of gripper linear velocity and gripper angular velocity
    Vector3d _velGMin, _angVelGMin; // the minimum of gripper linear velocity and gripper angular velocity
    Vector3d _velGCmd, _angVelGCmd; // expressed in BODY or WORLD frame

    // arm joint
    Vector6d _qAJMax, _dqAJMax; // the maximum of arm joint position and arm joint velocity
    Vector6d _qAJMin, _dqAJMin; // the minimum of arm joint position and arm joint velocity
    Vector6d _dqAJCmd;          // expressed in joint LOCAL frame

    // Generalized
    VecNq _posGenCmd;
    VecNv _velGenCmd;
};
