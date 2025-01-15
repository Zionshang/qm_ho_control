#include "foot_planner.hpp"

FootPlanner::FootPlanner(Estimator *est)
    : _est(est)
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

FootPlanner::~FootPlanner() {}

void FootPlanner::setGait(Vec3 vBd, Vec3 wBd)
{
    _vxyGoal = vBd.tail(2);
    _dYawGoal = wBd(2);
}

void FootPlanner::update(Vec34 &feetPosDes, Vec34 &feetVelDes)
{
    _phase = _est->getPhase();
    _contact = _est->getContact();
    if (_firstRun)
    {
        _startP = _est->getPosF();
        _firstRun = false;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (_contact(i) == 1)
        {
            if (_phase(i) < 0.5)
                _startP.col(i) = _est->getPosF().col(i);
            feetPosDes.col(i) = _startP.col(i);
            feetVelDes.col(i).setZero();
        }
        else
        {
            _endP.col(i) = calcFootholdPosition(i);
            feetPosDes.col(i) = calcReferenceFootPosition(i);
            feetVelDes.col(i) = calcReferenceFootVelocity(i);
        }
    }
}

Vec3 FootPlanner::calcFootholdPosition(int legID)
{
    // Translation in x,y axis
    _nextStep(0) = _est->getVelB()(0) * (1 - _phase(legID)) * _est->getTsw() + _est->getVelB()(0) * _est->getTst() / 2 + _kx * (_est->getVelB()(0) - _vxyGoal(0));
    _nextStep(1) = _est->getVelB()(1) * (1 - _phase(legID)) * _est->getTsw() + _est->getVelB()(1) * _est->getTst() / 2 + _ky * (_est->getVelB()(1) - _vxyGoal(1));
    _nextStep(2) = 0;

    // rotation about z axis
    _yaw = rotMat2RPY(_est->getRotB())(2);
    _nextYaw = _est->getAngVelB()(2) * (1 - _phase(legID)) * _est->getTsw() + _est->getAngVelB()(2) * _est->getTst() / 2 + _kyaw * (_dYawGoal - _est->getAngVelB()(2));
    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    Vec3 footholdPos;
    footholdPos = _est->getPosB() + _nextStep;
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
    return (end - start) * (1 - cos(phasePI)) / _est->getTsw();
}

double FootPlanner::cycloidZPosition(double start, double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * (1 - cos(phasePI)) / 2 + start;
}

double FootPlanner::cycloidZVelocity(double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * M_PI * sin(phasePI) / _est->getTsw();
}