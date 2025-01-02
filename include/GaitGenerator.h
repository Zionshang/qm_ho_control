#pragma once
#include "GaitSchedule.h"
#include "Estimator.h"

class GaitGenerator
{
public:
    GaitGenerator(Estimator *est);
    ~GaitGenerator();
    void setGait(Vec3 vBd, Vec3 wBd);
    void run(Vec34 &feetPosDes, Vec34 &feetVelDes);

private:
    Vec3 calFootholdPos(int legID);
    Vec3 getFootPosDes(int legID);
    Vec3 getFootVelDes(int legID);
    float cycloidXYPosition(float start, float end, float phase);
    float cycloidXYVelocity(float start, float end, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);

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

