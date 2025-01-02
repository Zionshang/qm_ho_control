#pragma once

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorCmd
{
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd()
    {
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowCmd
{
    MotorCmd motorLegCmd[12];

    MotorCmd motorArmCmd[6];

    void setLegTau(Vec12 tau, double tauMin = -200, double tauMax = 200)
    {
        for (int i = 0; i < 12; ++i)
        {
            if (std::isnan(tau(i)))
                printf("[ERROR] The setTau function meets Nan\n");
            motorLegCmd[i].tau = saturation(tau(i), tauMin, tauMax);
        }
    }

    void setArmTau(Vec6 tau, double tauMin = -200, double tauMax = 200)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (std::isnan(tau(i)))
                printf("[ERROR] The setTau function meets Nan\n");
            motorArmCmd[i].tau = saturation(tau(i), tauMin, tauMax);
        }
    }
};
