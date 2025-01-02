#pragma once

// enum class FSMStateName
// {
//     INVALID,
//     PASSIVE,
//     FIXEDSTAND,
//     FREESTAND,
//     TROT
// };

// enum class FSMMode
// {
//     NORMAL,
//     CHANGE
// };

enum class UserCommand
{
    A,
    B,
    X,
    Y
};

enum class GaitName
{
    STANCE,
    WALK,
    TROT,
    WALKING_TROT,
    RUNNING_TROT,
};

enum class WorkMode
{
    ARM_JOINT, 
    ARM_CARTESIAN_BODY,
    ARM_FIXED_BODY,
    ARM_FIXED_WORLD
};
