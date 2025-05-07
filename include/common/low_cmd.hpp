#pragma once

#include "common/types.hpp"
#include "utils/math_tools.hpp"

struct MotorCmd
{
    double q;
    double dq;
    double tau;
    double kp;
    double kd;

    MotorCmd()
    {
        q = 0;
        dq = 0;
        tau = 0;
        kp = 0;
        kd = 0;
    }
};

struct LowCmd
{
    MotorCmd motor_cmd_leg[12];
    MotorCmd motor_cmd_arm[6]; // todo: 不使用魔法数字
};
