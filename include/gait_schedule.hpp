#pragma once

#include "common/mathTypes.h"
#include "common/enumClass.h"
#include "common/LowState.h"
#include <iostream>
#include <vector>

struct Gait
{
    GaitName gait_name;
    double period;        // 步态周期(包含支撑腿周期和摆动腿周期的完整周期)
    double stance_ratio;  // 触底系数
    Vector4d bias;        // 偏移比例
    double period_stance; // 支撑腿周期
    double period_swing;  // 摆动腿周期

    Gait(GaitName gait_name = GaitName::STANCE) : gait_name(gait_name)
    {
        switch (gait_name)
        {
        case GaitName::STANCE:
            period = 0.4; // can be any value for STANCE
            stance_ratio = 1;
            bias << 0, 0, 0, 0;
            break;
        case GaitName::WALK:
            period = 0.3;
            stance_ratio = 0.75;
            bias << 0.25, 0.75, 0.5, 0;
            break;
        case GaitName::TROT:
            period = 0.4;
            stance_ratio = 0.5;
            bias << 0, 0.5, 0.5, 0;
            break;
        case GaitName::WALKING_TROT:
            period = 0.4;
            stance_ratio = 0.6;
            bias << 0, 0.5, 0.5, 0;
            break;
        case GaitName::RUNNING_TROT:
            period = 0.4;
            stance_ratio = 0.35;
            bias << 0, 0.5, 0.5, 0;
            break;
        }

        period_stance = period * stance_ratio;
        period_swing = period * (1 - stance_ratio);
    }
};

struct GaitList
{
    GaitList()
    {
        addGait(GaitName::STANCE);
        addGait(GaitName::WALK);
        addGait(GaitName::TROT);
        addGait(GaitName::WALKING_TROT);
        addGait(GaitName::RUNNING_TROT);
    }

    void addGait(GaitName gait_name)
    {
        gaits.push_back(Gait(gait_name));
    }

    const Gait &getGait(GaitName gait_name) const
    {
        for (const auto &gait : gaits)
        {
            if (gait.gait_name == gait_name)
                return gait;
        }
        throw std::runtime_error("Gait not found in gait list");
    }

    std::vector<Gait> gaits;
};

class GaitSchedule
{
public:
    GaitSchedule();
    ~GaitSchedule();
    void update(double currentT, GaitName target_gait_name);

    const Vector4d &phase() const { return phase_; }
    const Vector4i &contact() const { return contact_; }
    const Gait &current_gait() const { return gait_list_->getGait(current_gait_name_); }
    const double &period_stance() const { return current_gait().period_stance; }
    const double &period_swing() const { return current_gait().period_swing; }

private:
    void calcGaitPhase(double currentT);

    GaitName current_gait_name_;
    GaitList *gait_list_;
    double period_, stance_ratio_;
    Vector4d bias_;

    double time_current_;          // second
    double time_gait_start_;       // second
    double time_gait_passed_;      // second
    double time_switch_to_stance_; // second

    Vector4d time_normalized_; // normalize current time to the gait period
    Vector4i contact_;         // 1:contact  0:swting
    Vector4d phase_;           // progress of swing/stance as a proportion of swing/stace cycle [0,1]
    bool switch_to_stance_;
};
