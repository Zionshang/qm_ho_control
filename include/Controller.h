#pragma once

#include "common/LowCmd.h"
#include "Estimator.h"
#include "hierarchical_wbc.hpp"
#include "ctrl_component.hpp"
#include "pinocchio_interface.hpp"

class Controller
{
public:
    Controller(Estimator *est, LowCmd *lowCmd, PinocchioInterface *pin_interface);
    ~Controller();
    void run(const RobotState &robot_state, const RobotState &robot_state_ref,
             const Vector4i &contact);

private:
    LowState *_lowstate;
    Estimator *_est;
    LowCmd *_lowCmd;
    HierarchicalWbc *_hieraOpt;

    VectorXd _tau;
};