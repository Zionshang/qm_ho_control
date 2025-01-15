#include "foot_planner.hpp"

FootPlanner::FootPlanner()
{
    _gaitHeight = 0.04;
    _footOffset = 0.026;
    _firstRun = true;

    // used in caculation of foothold
    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;

    Vec24 posStill;                               // XY position when stance in still
    posStill << 0.2407, -0.2407, 0.2407, -0.2407, // clang-format off
                -0.138,  -0.138,  0.138,   0.138;  // clang-foramt on
    for (int i = 0; i < 4; ++i)
    {
        _feetRadius(i) = sqrt(pow(posStill(0, i), 2) + pow(posStill(1, i), 2));
        _feetInitAngle(i) = atan2(posStill(1, i), posStill(0, i));
    }
}

// TODO: haven't been used 
void FootPlanner::setGait(Vec3 vBd, Vec3 wBd)
{
    _vxyGoal = vBd.tail(2);
    _dYawGoal = wBd(2);
}

void FootPlanner::update(const BodyState &body_state, const Vector4d &phase, const Vector4i &contact,
                         const Matrix34d &pos_feet, double period_swing, double period_stance, 
                         Vec34 &feetPosDes, Vec34 &feetVelDes)
{
    _phase =  phase;
    _contact = contact;
    period_swing_ = period_swing;
    period_stance_ = period_stance;

    if (_firstRun)
    {
        _startP = pos_feet;
        _firstRun = false;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (_contact(i) == 1)
        {
            if (_phase(i) < 0.5)
                _startP.col(i) = pos_feet.col(i);
            feetPosDes.col(i) = _startP.col(i);
            feetVelDes.col(i).setZero();
        }
        else
        {
            _endP.col(i) = calcFootholdPosition(body_state, i);
            feetPosDes.col(i) = calcReferenceFootPosition(i);
            feetVelDes.col(i) = calcReferenceFootVelocity(i);
        }
    }
}

Vec3 FootPlanner::calcFootholdPosition(const BodyState &body_state, int legID)
{
    const Vec3 pos_body = body_state.pos;
    const Vec3 vel_body = body_state.vel;
    const RotMat rotmat_body = body_state.rotmat;
    const Vec3 angvel_body = body_state.angvel;

    // TODO: 是否需要改成相对于body系下
    // Translation in x,y axis
    _nextStep(0) = vel_body(0) * (1 - _phase(legID)) * period_swing_ + vel_body(0) * period_stance_ / 2 + _kx * (vel_body(0) - _vxyGoal(0));
    _nextStep(1) = vel_body(1) * (1 - _phase(legID)) * period_swing_ + vel_body(1) * period_stance_ / 2 + _ky * (vel_body(1) - _vxyGoal(1));
    _nextStep(2) = 0;

    // rotation about z axis
    _yaw = rotMat2RPY(rotmat_body)(2);
    _nextYaw = angvel_body(2) * (1 - _phase(legID)) * period_swing_ + angvel_body(2) * period_stance_ / 2 + _kyaw * (_dYawGoal - angvel_body(2));
    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    Vec3 footholdPos;
    footholdPos = pos_body + _nextStep;
    footholdPos(2) = _footOffset;

    return footholdPos;
}

Vec3 FootPlanner::calcReferenceFootPosition(int legID)
{
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(legID)(0), _endP.col(legID)(0), _phase(legID));
    footPos(1) = cycloidXYPosition(_startP.col(legID)(1), _endP.col(legID)(1), _phase(legID));
    footPos(2) = cycloidZPosition(_startP.col(legID)(2), _gaitHeight + _footOffset, _phase(legID));

    return footPos;
}

Vec3 FootPlanner::calcReferenceFootVelocity(int legID)
{
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(legID)(0), _endP.col(legID)(0), _phase(legID));
    footVel(1) = cycloidXYVelocity(_startP.col(legID)(1), _endP.col(legID)(1), _phase(legID));
    footVel(2) = cycloidZVelocity(_gaitHeight + _footOffset, _phase(legID));

    return footVel;
}

double FootPlanner::cycloidXYPosition(double start, double end, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
}

double FootPlanner::cycloidXYVelocity(double start, double end, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return (end - start) * (1 - cos(phasePI)) / period_swing_;
}

double FootPlanner::cycloidZPosition(double start, double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * (1 - cos(phasePI)) / 2 + start;
}

double FootPlanner::cycloidZVelocity(double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * M_PI * sin(phasePI) / period_swing_;
}