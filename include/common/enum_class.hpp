#pragma once

enum class GaitName
{
    STANCE,
    WALK,
    TROT,
    WALKING_TROT,
    RUNNING_TROT,
};

enum class ControllerType
{
    POSITION_STAND_UP,
    POSITION_LIE_DOWN,
    TORQUE_CONTROLLER,
    NONE,
};