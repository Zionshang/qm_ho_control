#pragma once

#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "common/projectPath.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

class Arm
{
public:
    Arm();
    ~Arm(){};
    void setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG);
    void setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG,
                           const Vec6 &dq, Vec3 &velG, Vec3 &angVelG);

private:
    pinocchio::Model _model;
    pinocchio::Data _data;
    int _idG;
};
