#pragma once
#include "common/mathTypes.h"

class QuadrupedLeg
{
public:
    QuadrupedLeg(int legID);
    ~QuadrupedLeg(){};
    Vec3 FK(const Vec3 &q);
    Vec3 IK(const Vec3 &posF);
    Mat3 Jacobian(const Vec3 &q);
    Vec3 FKDerivative(const Vec3 &q, const Vec3 &dq);
    Vec3 IKDerivative(const Vec3 &posF, const Vec3 &velF);

private:
    float _abadLink;  // abduction and adduction link length (after joint1)
    float _upperLink; // hip link length (after joint2)
    float _lowerLink; // knee link length (after joint3)
    float _sideSign;
};
