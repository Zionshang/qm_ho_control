#pragma once

#include "common/low_cmd.hpp"
#include "hierarchical_wbc.hpp"
#include "ctrl_component.hpp"
#include "pinocchio_interface.hpp"

class Controller
{
public:
    Controller(shared_ptr<PinocchioInterface> pin_interface);
    void run(const RobotState &robot_state, const RobotState &robot_state_ref,
             const Vector4i &contact, LowCmd &low_cmd);

private:
    shared_ptr<HierarchicalWbc> wbc_;
    VectorXd tau_;
};