#include "Controller.h"

Controller::Controller(Estimator *est, HighCmd *highCmd, LowCmd *lowCmd, WholeBodyDynamics *wbDyn)
    : _est(est), _lowCmd(lowCmd)
{
    _hieraOpt = new HierarchicalWbc(highCmd, wbDyn);
    _tau.setZero(18);
}

Controller::~Controller()
{
    delete _hieraOpt;
}

void Controller::run(const RobotState &robot_state, const Vector4i &contact)
{
    _hieraOpt->calTau(robot_state, contact, _tau);
    _lowCmd->setLegTau(_tau.head(12));
    _lowCmd->setArmTau(_tau.tail(6));
}
