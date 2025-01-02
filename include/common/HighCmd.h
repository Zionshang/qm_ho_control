#pragma once
#include "common/mathTypes.h"

struct HighCmd
{
    Vec3 posB;    // desired body position, expressed in WORLD frame
    Vec3 velB;    // desired body velocity, expressed in WORLD frame
    Quat quatB;   // desired body quaternion w.r.t. WORLD frame
    Vec3 angVelB; // desired body angular velocity, expressed in WORLD frame
    Vec34 posF;   // desired all feet position, expressed in WORLD frame
    Vec34 velF;   // desired all feet velocity, expressed in WORLD frame
   Vec3 posCoM;  // desired body position, expressed in WORLD frame
    Vec3 velCoM;  // desired body velocity, expressed in WORLD frame

    Vec3 posG;    // desired gripper position, expressed in WORLD frame
    Vec3 velG;    // desired gripper velocity, expressed in WORLD frame
    Quat quatG;   // desired gripper quaternion
    Vec3 angVelG; // desired gripper angular velocity, expressed in WORLD frame
    Vec6 qAJ;     // desired arm joint position, expressed in LOCAL JOINT frame
    Vec6 dqAJ;    // desired arm joint velocity, expressed in LOCAL JOINT frame

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
