#include "controller.hpp"

Controller::Controller(PinocchioInterface *pin_interface)
{
    wbc_ = new HierarchicalWbc(pin_interface);
    tau_.setZero(pin_interface->nv() - 6);
}

Controller::~Controller()
{
    delete wbc_;
}

void Controller::run(const RobotState &robot_state, const RobotState &robot_state_ref,
                     const Vector4i &contact, LowCmd &low_cmd)
{
    wbc_->calTau(robot_state, robot_state_ref, contact, tau_);
    low_cmd.setLegTau(tau_.head(12));
    low_cmd.setArmTau(tau_.tail(6));
}
