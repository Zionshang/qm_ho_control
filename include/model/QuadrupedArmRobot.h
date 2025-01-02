#pragma once
#include "common/mathTypes.h"
#include "model/QuadrupedLeg.h"
#include "model/Arm.h"

class QuadrupedArmRobot
{
public:
    QuadrupedArmRobot();
    ~QuadrupedArmRobot();

    Vec3 calFootPos(const Vec3 &q, int id);
    Vec34 calFeetPos(const Vec34 &q);
    Vec3 calFootVel(const Vec3 &q, const Vec3 &dq, int id);
    Vec34 calFeetVel(const Vec34 &q, const Vec34 &dq);
    Vec34 calQLeg(const Vec34 &posF2B_B);
    Vec34 calDqLeg(const Vec34 &q, const Vec34 &velF2B_B);

    void setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG);
    void setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG,
                           const Vec6 &dq, Vec3 &velG, Vec3 &angVelG);

private:
    QuadrupedLeg *_Legs[4];
    Arm *_arm;
    Vec3 _posS2B;       // position of SHOULDER frame w.r.t. BODY frame, expressed in BODY frame.
    Vec34 _posA2B;      // position of ABAD frame w.r.t BODY frame, expressed in BODY frame.
    float _abadOffsetX; // offset of ABAD frame in x axis
    float _abadOffsetY; // offset of ABAD frame in y axis
};
