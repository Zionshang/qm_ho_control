#pragma once

#include "common/HighCmd.h"
#include "common/LowState.h"
#include "foot_planner.hpp"

class Planner
{
public:
    Planner(HighCmd *highCmd, Estimator *est, LowState *lowState, WholeBodyDynamics *wbDyn);
    ~Planner();
    void setDesiredTraj(const RobotState &robot_state, const GaitState &gait_state);
    void showDemo(const RobotState &robot_state, const GaitState &gait_state);
    void printDesiredTraj();
    void showFrontMaxJointVelDemo(const RobotState &robot_state, const GaitState &gait_state);
    void showSideMaxJointVelDemo(const RobotState &robot_state, const GaitState &gait_state);
    void showPickingDemo(const RobotState &robot_state, const GaitState &gait_state);
    
private:
    void bodyPlan();
    void gripperPlan();
    void armJointPlan();
    void comPlan();

    HighCmd *_highCmd;
    FootPlanner *_gaitGen;
    Estimator *_est;
    LowState *_lowState;
    WholeBodyDynamics *_wbDyn;

    double _dt;
    RotMat _rotB_d;     // desired rotation matrix of BODY reletive to GLOBAL
    bool _isModeChange; // Mode change:1  Mode not change: 0
    WorkMode _lastWorkMode;

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
