#include "Controller.h"

Controller::Controller(Estimator *est, LowCmd *lowCmd, PinocchioInterface *pin_interface)
    : _est(est), _lowCmd(lowCmd)
{
    _hieraOpt = new HierarchicalWbc(pin_interface);
    _tau.setZero(18);
}

Controller::~Controller()
{
    delete _hieraOpt;
}

void Controller::run(const RobotState &robot_state, const RobotState &robot_state_ref,
                     const Vector4i &contact)
{
    _hieraOpt->calTau(robot_state, robot_state_ref, contact, _tau);
    _lowCmd->setLegTau(_tau.head(12));
    _lowCmd->setArmTau(_tau.tail(6));
}
