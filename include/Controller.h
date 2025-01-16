#pragma once

#include "common/LowCmd.h"
#include "Estimator.h"
#include "hierarchical_wbc.hpp"
#include "ctrl_component.hpp"

class Controller
{
public:
    Controller(Estimator *est, HighCmd *highCmd, LowCmd *lowCmd, WholeBodyDynamics *wbDyn);
    ~Controller();
    void run(const RobotState &robot_state, const Vector4i &contact);

private:
    LowState *_lowstate;
    Estimator *_est;
    LowCmd *_lowCmd;
    HierarchicalWbc *_hieraOpt;

    Vec18 _tau;
};