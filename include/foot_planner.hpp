#pragma once
#include "gait_schedule.hpp"
#include "Estimator.h"

class FootPlanner
{
public:
    FootPlanner(Estimator *est);
    void setGait(Vec3 vBd, Vec3 wBd);
    void update(const BodyState &body_state, const Vector4d &phase, const Vector4i &contact,
                Vec34 &feetPosDes, Vec34 &feetVelDes);

private:
    Vector3d calcFootholdPosition(const BodyState &body_state, int legID);
    Vector3d calcReferenceFootPosition(int legID);
    Vector3d calcReferenceFootVelocity(int legID);
    double cycloidXYPosition(double start, double end, double phase);
    double cycloidXYVelocity(double start, double end, double phase);
    double cycloidZPosition(double startZ, double height, double phase);
    double cycloidZVelocity(double height, double phase);

    Estimator *_est;

    Matrix34d _startP, _endP;
    double _gaitHeight;
    double _footOffset; // Radius of sphere at foot
    bool _firstRun;
    Vector4d _phase;
    VecInt4 _contact;

    // foothold
    double _kx, _ky, _kyaw;
    Vec2 _vxyGoal;
    double _dYawGoal, _nextYaw, _yaw;
    Vector4d _feetRadius, _feetInitAngle;
    Vector3d _nextStep;
};
