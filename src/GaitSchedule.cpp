#include "GaitSchedule.h"

GaitSchedule::GaitSchedule(GaitName gaitName)
{
    _gait = new Gait(gaitName);
    _period = _gait->period;
    _stRatio = _gait->stancePhaseRatio;
    _bias = _gait->bias;
    _startT = 0.0;
}

GaitSchedule::~GaitSchedule()
{
    delete _gait;
}

void GaitSchedule::run(double currentT)
{
    // _startT = currentT;
    calcPhase(currentT);
}

double GaitSchedule::getTst() const
{
    return _period * _stRatio;
}

double GaitSchedule::getTsw() const
{
    return _period * (1 - _stRatio);
}

void GaitSchedule::calcPhase(double currentT)
{
    if (_gait->gaitName == GaitName::STANCE)
    {
        _contact.setOnes();
        _phase << 0.5, 0.5, 0.5, 0.5;
    }
    else
    {
        _passT = currentT - _startT;
        for (int i = 0; i < 4; i++)
        {
            // Normalize current time to the gait period in which current time is located
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio)
            {
                _contact(i) = 1;
                _phase(i) = _normalT(i) / _stRatio;
            }
            else
            {
                _contact(i) = 0;
                _phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
}