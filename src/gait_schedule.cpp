#include "gait_schedule.hpp"

GaitSchedule::GaitSchedule()
{
    gait_list_ = new GaitList();
    current_gait_name_ = GaitName::STANCE;
    time_gait_start_ = 0.0;
    switch_to_stance_ = false;
    time_switch_to_stance_ = 0.0;
}

GaitSchedule::~GaitSchedule()
{
    delete gait_list_;
}

void GaitSchedule::update(double currentT, GaitName target_gait_name)
{

    if (switch_to_stance_)
    {
        if (currentT - time_switch_to_stance_ >= 0.5)
        {
            switch_to_stance_ = false;
            current_gait_name_ = target_gait_name;
            time_gait_start_ = currentT;
        }
    }
    else if (target_gait_name != current_gait_name_)
    {
        switch_to_stance_ = true;
        time_switch_to_stance_ = currentT;
        current_gait_name_ = GaitName::STANCE;
    }
    
    const auto &current_gait = gait_list_->getGait(current_gait_name_);
    period_ = current_gait.period;
    stance_ratio_ = current_gait.stance_ratio;
    bias_ = current_gait.bias;

    calcGaitPhase(currentT);
}

void GaitSchedule::calcGaitPhase(double currentT)
{
    time_gait_passed_ = currentT - time_gait_start_;
    for (int i = 0; i < 4; i++)
    {
        time_normalized_(i) = fmod(time_gait_passed_ + period_ - period_ * bias_(i), period_) / period_;
        if (time_normalized_(i) < stance_ratio_)
        {
            contact_(i) = 1;
            phase_(i) = time_normalized_(i) / stance_ratio_;
        }
        else
        {
            contact_(i) = 0;
            phase_(i) = (time_normalized_(i) - stance_ratio_) / (1 - stance_ratio_);
        }
    }
}
