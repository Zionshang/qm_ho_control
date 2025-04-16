#include "position_controller.hpp"

PositionController::PositionController(double dt)
    : dt_(dt)
{
    stand_flag_ = false;
    lie_flag_ = false;
    arm_flag_ = false;

    lie_joint_pos_ << 0, 0, 0, 0,
        1.3, 1.3, 1.3, 1.3,
        -2.6, -2.6, -2.6, -2.6;
    stand_joint_pos_ << 0.0, 0.0, 0.0, 0.0,
        0.72, 0.72, 0.72, 0.72,
        -1.44, -1.44, -1.44, -1.44;
    default_arm_joint_pos_.setZero();

    stand_time_ = 0;
    lie_time_ = 0;
    arm_time_ = 0;
    period_ = 10;

    stand_spline0_.resize(12);
    stand_spline1_.resize(12);
    lie_spline_.resize(12);
    arm_spline_.resize(6);
}

void PositionController::updateToStandUp(const Matrix34d &joint_pos, LowCmd &low_cmd)
{
    if (!stand_flag_)
    {
        stand_time_ = 0;
        setStandUp(joint_pos);
    }
    else
    {
        stand_time_ += dt_;
        for (int i = 0; i < 12; i++)
        {
            if (stand_time_ < period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_spline0_[i].position(stand_time_);
                low_cmd.motor_cmd_leg[i].dq = stand_spline0_[i].velocity(stand_time_);
            }
            else if (stand_time_ >= period_ && stand_time_ < period_ + period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_spline1_[i].position(stand_time_);
                low_cmd.motor_cmd_leg[i].dq = stand_spline1_[i].velocity(stand_time_);
            }
            else if (stand_time_ >= period_ + period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_joint_pos_(i);
                low_cmd.motor_cmd_leg[i].dq = 0;
            }
            low_cmd.motor_cmd_leg[i].kp = 0.0;
            low_cmd.motor_cmd_leg[i].kd = 0;
            low_cmd.motor_cmd_leg[i].tau = 0;
            low_cmd.motor_cmd_leg[i].dq = 0;
        }
    }
}

void PositionController::updateToLieDown(const Matrix34d &joint_pos, LowCmd &low_cmd)
{
    if (!lie_flag_)
    {
        lie_time_ = 0;
        setLieDown(joint_pos);
    }
    else
    {
        lie_time_ += dt_;
        for (int i = 0; i < 12; i++)
        {
            if (lie_time_ < period_)
            {
                low_cmd.motor_cmd_leg[i].q = lie_spline_[i].position(lie_time_);
                low_cmd.motor_cmd_leg[i].dq = lie_spline_[i].velocity(lie_time_);
            }
            else if (lie_time_ >= period_)
            {
                low_cmd.motor_cmd_leg[i].q = lie_joint_pos_(i);
                low_cmd.motor_cmd_leg[i].dq = 0;
            }

            low_cmd.motor_cmd_leg[i].kp = 0.0;
            low_cmd.motor_cmd_leg[i].kd = 0;
            low_cmd.motor_cmd_leg[i].tau = 0;
            low_cmd.motor_cmd_leg[i].dq = 0;
        }
    }
}

void PositionController::updateArmToDefault(const Vector6d &joint_pos, LowCmd &low_cmd)
{
    if (!arm_flag_)
    {
        arm_time_ = 0;
        setArmDefault(joint_pos);
    }

    for (int i = 0; i < 6; i++)
    {
        arm_time_ += dt_;
        if (arm_time_ < period_)
        {
            low_cmd.motor_cmd_arm[i].q = arm_spline_[i].position(arm_time_);
            low_cmd.motor_cmd_arm[i].dq = arm_spline_[i].velocity(arm_time_);
        }
        else if (arm_time_ >= period_)
        {
            low_cmd.motor_cmd_arm[i].q = default_arm_joint_pos_(i);
            low_cmd.motor_cmd_arm[i].dq = 0;
        }

        low_cmd.motor_cmd_arm[i].kp = 0;
        low_cmd.motor_cmd_arm[i].kd = 5;
        low_cmd.motor_cmd_arm[i].tau = 0;
        low_cmd.motor_cmd_arm[i].dq = 0;
    }
}

void PositionController::setStandUp(const Matrix34d &current_pos)
{
    stand_flag_ = true;
    CubicSpline::SplineNode start, end;

    for (int i = 0; i < 12; i++)
    {
        start.time = 0;
        start.position = current_pos(i);
        start.velocity = 0;

        end.time = period_;
        end.position = lie_joint_pos_(i);
        end.velocity = 0;
        stand_spline0_[i] = CubicSpline(start, end);
    }

    for (int i = 0; i < 12; i++)
    {
        start.time = period_;
        start.position = lie_joint_pos_(i);
        start.velocity = 0;

        end.time = period_ + period_;
        end.position = stand_joint_pos_(i);
        end.velocity = 0;
        stand_spline1_[i] = CubicSpline(start, end);
    }
}

void PositionController::setLieDown(const Matrix34d &current_pos)
{
    lie_flag_ = true;
    CubicSpline::SplineNode start, end;

    for (int i = 0; i < 12; i++)
    {
        start.time = 0;
        start.position = current_pos(i);
        start.velocity = 0;

        end.time = period_;
        end.position = lie_joint_pos_(i);
        end.velocity = 0;
        lie_spline_[i] = CubicSpline(start, end);
    }
}

void PositionController::setArmDefault(const Vector6d &current_pos)
{
    arm_flag_ = true;
    CubicSpline::SplineNode start, end;
    for (int i = 0; i < 6; i++)
    {
        start.time = 0;
        start.position = current_pos(i);
        start.velocity = 0;

        end.time = period_;
        end.position = default_arm_joint_pos_(i);
        end.velocity = 0;
        arm_spline_[i] = CubicSpline(start, end);
    }
}
