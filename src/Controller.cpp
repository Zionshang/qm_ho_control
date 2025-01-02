#include "Controller.h"

Controller::Controller(Estimator *est, HighCmd *highCmd, LowCmd *lowCmd, WholeBodyDynamics *wbDyn)
    : _est(est), _lowCmd(lowCmd)
{
    _hieraOpt = new HoControl(highCmd, est, wbDyn);
    _tau.setZero();
}

Controller::~Controller()
{
    delete _hieraOpt;
}

void Controller::run()
{
    _hieraOpt->calTau(_tau);
    _lowCmd->setLegTau(_tau.head(12));
    _lowCmd->setArmTau(_tau.tail(6));
}
