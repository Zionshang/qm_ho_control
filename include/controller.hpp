#pragma once

#include "common/low_cmd.hpp"
#include "hierarchical_wbc.hpp"
#include "ctrl_component.hpp"
#include "pinocchio_interface.hpp"

class Controller
{
public:
    Controller(PinocchioInterface *pin_interface);
    ~Controller();
    void run(const RobotState &robot_state, const RobotState &robot_state_ref,
             const Vector4i &contact, LowCmd &low_cmd);

private:
    HierarchicalWbc *wbc_;
    VectorXd tau_;
};