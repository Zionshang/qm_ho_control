#pragma once
#include "common/mathTypes.h"
#include "common/LowState.h"
#include "model/WholeBodyDynamics.h"
#include "pinocchio_interface.hpp"
#include "ctrl_component.hpp"

class Estimator
{
public:
    Estimator(LowState *lowState, PinocchioInterface *pin_interface);
    void update(RobotState &robot_state);

    // const BodyState &body_state() const { return body_state_; }

    // const VectorXd &pos_gen() const { return _posGen; }
    // const VectorXd &vel_gen() const { return _velGen; }

    // Vector3d getPosB() const { return body_state_.pos; }       // get position of BODY, expressed in WORLD frame
    // Vector3d getVelB() const { return body_state_.vel; }       // get velocity of BODY, expressed in WORLD frame
    // Quaternion getQuatB() const { return body_state_.quat; }     // get quaternion of BODY relative to WORLD
    // RotMat getRotB() const { return body_state_.rotmat; }  // get rotation matrix of BODY relative to WORLD
    // Vector3d getAngVelB() const { return body_state_.angvel; } // get angular velocity of BODY, expressed in WORLD frame

    // // CoM
    // Vector3d getPosCoM() const { return _posCoM; } // get position of CoM, expressed in WORLD frame
    // Vector3d getVelCoM() const { return _velCoM; } // get velocity of CoM, expressed in WORLD frame

    // // foot
    // Matrix34d getQLeg() const { return _qLeg; }            // get joint position of four legs
    // Matrix34d getDqLeg() const { return _dqLeg; }          // get joint velocity of four legs
    // Vector3d getPosF(int i) const { return _posF.col(i); } // get position of id foot, expressed in WORLD frame
    // Vector3d getVelF(int i) const { return _velF.col(i); } // get velocity of id foot, expressed in WORLD frame
    // Matrix34d getPosF() const { return _posF; }            // get position of four feet, expressed in WORLD frame
    // Matrix34d getVelF() const { return _velF; }            // get velocity of four feet, expressed in WORLD frame

    // // gripper
    // Vector6d getQArm() const { return _qArm; }        // get joint position of arm
    // Vector6d getDqArm() const { return _dqArm; }      // get joint velocity of arm
    Vector3d getPosG() const { return _posG; }        // get position of GRIPPER, expressed in WORLD frame
    Vector3d getVelG() const { return _velG; };       // get velocity of GRIPPER, expressed in WORLD frame
    Vector3d getAngVelG() const { return _angVelG; }; // get angular velocity of GRIPPER, expressed in WORLD frame
    Quaternion getQuatG() const { return _quatG; };     // get quaternion of GRIPPER relative to WORLD frame
    RotMat getRotG() const { return _rotG; }      // get rotation matrix of GRIPPER relative to WORLD frame

    double getTimeStep() const { return _lowState->getTimeStep(); }
    double getCurrentTime() const { return _lowState->getCurrentTime(); }
    void printState();

private:
    LowState *_lowState;
    PinocchioInterface *pin_interface_;

    // // // CoM
    // // VectorXd _posGen;
    // // VectorXd _velGen;
    // Vector3d _posCoM, _velCoM; // position and velocity CoM, expressed in WORLD frame

    // // leg & foot
    // Matrix34d _qLeg, _dqLeg;    // joint position and velocity of leg
    // Matrix34d _posF, _velF;     // position and velocity of FOOT, expressed in WORLD frame
    // Matrix34d _posF2B, _velF2B; // position and velocity of FOOT, expressed in WORLD frame

    // gait
    Vector4d _phase;      // progress of swing/stance as a proportion of swing/stace cycle [0,1]
    Vector4i _contact; // 1:contact  0:swting
    double T_sw_;
    double T_st_;

    // arm & gripper
    // Vector6d _qArm, _dqArm;          // joint position and velocity of arm
    Quaternion _quatG;                 // quaternion of GRIPER reletive to GLOBAL
    Vector3d _posG, _velG, _angVelG; // position, velocity and angular velocity of GRIPER, expressed in WORLD frame
    RotMat _rotG;                // rotation matrix of GRIPER reletive to WORLD
};
