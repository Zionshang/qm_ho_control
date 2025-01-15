#pragma once
#include "gait_schedule.hpp"
#include "Estimator.h"

class FootPlanner
{
public:
    FootPlanner(Estimator *est);
    ~FootPlanner();
    void setGait(Vec3 vBd, Vec3 wBd);
    void update(Vec34 &feetPosDes, Vec34 &feetVelDes);

private:
    Vec3 calcFootholdPosition(int legID);
    Vec3 calcReferenceFootPosition(int legID);
    Vec3 calcReferenceFootVelocity(int legID);
    double cycloidXYPosition(double start, double end, double phase);
    double cycloidXYVelocity(double start, double end, double phase);
    double cycloidZPosition(double startZ, double height, double phase);
    double cycloidZVelocity(double height, double phase);

    Estimator *_est;

    Vec34 _startP, _endP;
    double _gaitHeight;
    double _footOffset; // Radius of sphere at foot
    bool _firstRun;
    Vec4 _phase;
    VecInt4 _contact;

    // foothold
    double _kx, _ky, _kyaw;
    Vec2 _vxyGoal;
    double _dYawGoal, _nextYaw, _yaw;
    Vec4 _feetRadius, _feetInitAngle;
    Vec3 _nextStep;
};

