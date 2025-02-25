#pragma once

#include "common/math_types.hpp"
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
    MotorCmd motor_cmd_arm[6];

    void setLegTau(Vector12d tau, double tau_min = -200, double tau_max = 200)
    {
        for (int i = 0; i < 12; ++i)
        {
            if (std::isnan(tau(i)))
                printf("[ERROR] The setTau function meets Nan\n");
            motor_cmd_leg[i].tau = saturation(tau(i), tau_min, tau_max);
        }
    }

    void setArmTau(Vector6d tau, double tau_min = -200, double tau_max = 200)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (std::isnan(tau(i)))
                printf("[ERROR] The setTau function meets Nan\n");
            motor_cmd_arm[i].tau = saturation(tau(i), tau_min, tau_max);
        }
    }
};
