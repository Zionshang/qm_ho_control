#pragma once
#include "common/types.hpp"
#include "common/low_state.hpp"
#include "ctrl_component.hpp"
#include "webots_interface.hpp"
#include <iostream>

class CheatEstimator
{
public:
    CheatEstimator(shared_ptr<PinocchioInterface> pin_interface);
    void update(const webots::Node *cheat_node, const LowState &low_state, RobotState &robot_state);

private:
    shared_ptr<PinocchioInterface> pin_interface_;
};
