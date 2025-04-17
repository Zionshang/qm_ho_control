#include "controller.hpp"

Controller::Controller(shared_ptr<PinocchioInterface> pin_interface)
{
    wbc_ = std::make_shared<HierarchicalWbc>(pin_interface);
    tau_.setZero(pin_interface->nv() - 6);
}

void Controller::run(const RobotState &robot_state, const RobotState &robot_state_ref,
                     const Vector4i &contact, LowCmd &low_cmd)
{
    wbc_->calTau(robot_state, robot_state_ref, contact, tau_);

    // todo: 是否需要在此处限制关节力矩的大小？
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd_leg[i].kp = 0;
        low_cmd.motor_cmd_leg[i].kd = 0;
        low_cmd.motor_cmd_leg[i].q = 0;
        low_cmd.motor_cmd_leg[i].dq = 0;
        low_cmd.motor_cmd_leg[i].tau = tau_(i);
    }

    for (int i = 0; i < 5; i++)
    {
        low_cmd.motor_cmd_arm[i].kp = 0;
        low_cmd.motor_cmd_arm[i].kd = 0;
        low_cmd.motor_cmd_arm[i].q = 0;
        low_cmd.motor_cmd_arm[i].dq = 0;
        low_cmd.motor_cmd_arm[i].tau = tau_(12 + i);
    }
}
