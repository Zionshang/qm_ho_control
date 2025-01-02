#pragma once


#include "common/mathTypes.h"
#include "common/enumClass.h"
#include "common/LowState.h"
#include <iostream>

struct Gait
{
    GaitName gaitName;
    double period;           // 步态周期
    double stancePhaseRatio; // 触底系数
    Vec4 bias;               // 偏移比例

    Gait(GaitName gaitName) : gaitName(gaitName)
    {
        if (gaitName == GaitName::WALK)
        {
            period = 0.3;
            stancePhaseRatio = 0.75;
            bias << 0.25, 0.75, 0.5, 0;
        }
        else if (gaitName == GaitName::TROT)
        {
            period = 0.4;
            stancePhaseRatio = 0.5;
            bias << 0, 0.5, 0.5, 0;
        }
        else if (gaitName == GaitName::WALKING_TROT)
        {
            period = 0.4;
            stancePhaseRatio = 0.6;
            bias << 0, 0.5, 0.5, 0;
        }
        else if (gaitName == GaitName::RUNNING_TROT)
        {
            period = 0.4;
            stancePhaseRatio = 0.35;
            bias << 0, 0.5, 0.5, 0;
        }
    }
};

class GaitSchedule
{
public:
    GaitSchedule(GaitName gaitName);
    ~GaitSchedule();
    void run(double currentT);
    Vec4 getPhase() const { return _phase; }
    VecInt4 getContact() const { return _contact; }
    double getTst() const;
    double getTsw() const;

private:
    void calcPhase(double currentT);

    Gait *_gait;
    double _period, _stRatio;
    Vec4 _bias;
    double _currentT, _startT, _passT; // second
    Vec4 _normalT;                     // normalize current time to the gait period
    VecInt4 _contact;                  // 1:contact  0:swting
    Vec4 _phase;                       // progress of swing/stance as a proportion of swing/stace cycle [0,1]
};

