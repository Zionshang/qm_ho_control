#pragma once

#include "common/HighCmd.h"
#include "common/LowState.h"
#include "foot_planner.hpp"

class Planner
{
public:
    Planner(HighCmd *highCmd, Estimator *est, LowState *lowState, WholeBodyDynamics *wbDyn);
    ~Planner();
    void setDesiredTraj(const GaitState &gait_state);
    void showDemo(const GaitState &gait_state);
    void printDesiredTraj();
    void showFrontMaxJointVelDemo(const GaitState &gait_state);
    void showSideMaxJointVelDemo(const GaitState &gait_state);
    void showPickingDemo(const GaitState &gait_state);
    
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
    Vec3 _velBMax, _angVelBMax; // the maximum of body linear velocity and body angular velocity
    Vec3 _velBMin, _angVelBMin; // the minimum of body linear velocity and body angular velocity
    Vec3 _velBCmd, _angVelBCmd; // expressed in BODY frame

    // gripper
    Vec3 _velGMax, _angVelGMax; // the maximum of gripper linear velocity and gripper angular velocity
    Vec3 _velGMin, _angVelGMin; // the minimum of gripper linear velocity and gripper angular velocity
    Vec3 _velGCmd, _angVelGCmd; // expressed in BODY or WORLD frame

    // arm joint
    Vec6 _qAJMax, _dqAJMax; // the maximum of arm joint position and arm joint velocity
    Vec6 _qAJMin, _dqAJMin; // the minimum of arm joint position and arm joint velocity
    Vec6 _dqAJCmd;          // expressed in joint LOCAL frame

    // Generalized
    VecNq _posGenCmd;
    VecNv _velGenCmd;
};
