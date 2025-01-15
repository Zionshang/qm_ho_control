#pragma once
#include "common/mathTypes.h"
#include "common/LowState.h"
#include "model/WholeBodyDynamics.h"
#include "pinocchio_interface.hpp"

class Estimator
{
public:
    Estimator(LowState *lowState, PinocchioInterface *pin_interface);
    void update(const Vec4 &phase, const Vector4i &contact, double T_sw, double T_st);

    // body
    Vec3 getPosB() const { return _posB; }       // get position of BODY, expressed in WORLD frame
    Vec3 getVelB() const { return _velB; }       // get velocity of BODY, expressed in WORLD frame
    Quat getQuatB() const { return _quatB; }     // get quaternion of BODY relative to WORLD
    RotMat getRotB() const { return _rotB; }     // get rotation matrix of BODY relative to WORLD
    Vec3 getAngVelB() const { return _angVelB; } // get angular velocity of BODY, expressed in WORLD frame

    // CoM
    Vec3 getPosCoM() const { return _posCoM; } // get position of CoM, expressed in WORLD frame
    Vec3 getVelCoM() const { return _velCoM; } // get velocity of CoM, expressed in WORLD frame

    // foot
    Vec34 getQLeg() const { return _qLeg; }            // get joint position of four legs
    Vec34 getDqLeg() const { return _dqLeg; }          // get joint velocity of four legs
    Vec3 getPosF(int i) const { return _posF.col(i); } // get position of id foot, expressed in WORLD frame
    Vec3 getVelF(int i) const { return _velF.col(i); } // get velocity of id foot, expressed in WORLD frame
    Vec34 getPosF() const { return _posF; }            // get position of four feet, expressed in WORLD frame
    Vec34 getVelF() const { return _velF; }            // get velocity of four feet, expressed in WORLD frame

    // gait
    Vec4 getPhase() const { return _phase; }        // get progress of swing/stance as a proportion of swing/stace cycle [0,1]
    VecInt4 getContact() const { return _contact; } // get contact state. 1:contact  0:swting
    double getTsw() const { return T_sw_; }         // get peroid of swing phase
    double getTst() const { return T_st_; }         // get peroid of stance phase

    // gripper
    Vec6 getQArm() const { return _qArm; }        // get joint position of arm
    Vec6 getDqArm() const { return _dqArm; }      // get joint velocity of arm
    Vec3 getPosG() const { return _posG; }        // get position of GRIPPER, expressed in WORLD frame
    Vec3 getVelG() const { return _velG; };       // get velocity of GRIPPER, expressed in WORLD frame
    Vec3 getAngVelG() const { return _angVelG; }; // get angular velocity of GRIPPER, expressed in WORLD frame
    Quat getQuatG() const { return _quatG; };     // get quaternion of GRIPPER relative to WORLD frame
    RotMat getRotG() const { return _rotG; }      // get rotation matrix of GRIPPER relative to WORLD frame

    WorkMode getWorkMode() const { return _workMode; }
    double getTimeStep() const { return _lowState->getTimeStep(); }
    double getCurrentTime() const { return _lowState->getCurrentTime(); }
    void printState();

private:
    LowState *_lowState;
    PinocchioInterface *pin_interface_;

    // body
    Vec3 _posB, _velB; // position and velocity BODY, expressed in WORLD frame
    RotMat _rotB;      // rotation matrix of BODY relative to WORLD
    Quat _quatB;       // quaternion of BODY relative to WORLD
    Vec3 _angVelB;     // angular velocity of BODY, expressed in WORLD frame

    // CoM
    VecNq _posGen;
    VecNv _velGen;
    Vec3 _posCoM, _velCoM; // position and velocity CoM, expressed in WORLD frame

    // leg & foot
    Vec34 _qLeg, _dqLeg;    // joint position and velocity of leg
    Vec34 _posF, _velF;     // position and velocity of FOOT, expressed in WORLD frame
    Vec34 _posF2B, _velF2B; // position and velocity of FOOT, expressed in WORLD frame

    // gait
    Vec4 _phase;      // progress of swing/stance as a proportion of swing/stace cycle [0,1]
    VecInt4 _contact; // 1:contact  0:swting
    double T_sw_;
    double T_st_;

    // arm & gripper
    Vec6 _qArm, _dqArm;          // joint position and velocity of arm
    Quat _quatG;                 // quaternion of GRIPER reletive to GLOBAL
    Vec3 _posG, _velG, _angVelG; // position, velocity and angular velocity of GRIPER, expressed in WORLD frame
    RotMat _rotG;                // rotation matrix of GRIPER reletive to WORLD

    // control
    WorkMode _workMode;
};
