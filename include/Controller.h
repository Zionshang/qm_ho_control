#pragma once

#include "common/LowCmd.h"
#include "Estimator.h"
#include "HoControl.h"

class Controller
{
public:
    Controller(Estimator *est, HighCmd *highCmd, LowCmd *lowCmd, WholeBodyDynamics *wbDyn);
    ~Controller();
    void run();

private:
    LowState *_lowstate;
    Estimator *_est;
    LowCmd *_lowCmd;
    HoControl *_hieraOpt;

    Vec18 _tau;
};