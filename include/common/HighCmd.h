#pragma once
#include "common/mathTypes.h"

struct HighCmd
{
    Vector3d posB;    // desired body position, expressed in WORLD frame
    Vector3d velB;    // desired body velocity, expressed in WORLD frame
    Quaternion quatB;   // desired body quaternion w.r.t. WORLD frame
    Vector3d angVelB; // desired body angular velocity, expressed in WORLD frame
    Matrix34d posF;   // desired all feet position, expressed in WORLD frame
    Matrix34d velF;   // desired all feet velocity, expressed in WORLD frame
   Vector3d posCoM;  // desired body position, expressed in WORLD frame
    Vector3d velCoM;  // desired body velocity, expressed in WORLD frame

    Vector3d posG;    // desired gripper position, expressed in WORLD frame
    Vector3d velG;    // desired gripper velocity, expressed in WORLD frame
    Quaternion quatG;   // desired gripper quaternion
    Vector3d angVelG; // desired gripper angular velocity, expressed in WORLD frame
    Vector6d qAJ;     // desired arm joint position, expressed in LOCAL JOINT frame
    Vector6d dqAJ;    // desired arm joint velocity, expressed in LOCAL JOINT frame

    HighCmd()
    {
        posB << 0.0, 0.0, 0.42;
        quatB << 0.0, 0.0, 0.0, 1.0;

        posCoM << 0.0317053, 0.00152422, 0.440351;
        velCoM.setZero();

        velB.setZero();
        angVelB.setZero();
        posF.setZero();
        velF.setZero();

        posG << 0.38, 0, 0.6405;
        quatG << 0.0, 0.0, 0.0, 1.0;

        velG.setZero();
        angVelG.setZero();
        qAJ.setZero();
        dqAJ.setZero();
    }
};
