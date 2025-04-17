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
    default_arm_joint_pos_.setZero(5); // todo: 删除魔法数字
    default_arm_joint_pos_ << 0, -1.57, 2.88, 0.26, 0;

    time_ = 0;
    period_ = 1;

    stand_spline0_.resize(12);
    stand_spline1_.resize(12);
    lie_spline_.resize(12);
    arm_spline_.resize(5);
}

void PositionController::updateToStandUp(const Matrix34d &joint_pos, LowCmd &low_cmd)
{
    if (!stand_flag_)
    {
        time_ = 0;
        setStandUp(joint_pos);
    }
    else
    {
        time_ += dt_;
        for (int i = 0; i < 12; i++)
        {
            if (time_ < period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_spline0_[i].position(time_);
                low_cmd.motor_cmd_leg[i].dq = stand_spline0_[i].velocity(time_);
            }
            else if (time_ >= period_ && time_ < period_ + period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_spline1_[i].position(time_);
                low_cmd.motor_cmd_leg[i].dq = stand_spline1_[i].velocity(time_);
            }
            else if (time_ >= period_ + period_)
            {
                low_cmd.motor_cmd_leg[i].q = stand_joint_pos_(i);
                low_cmd.motor_cmd_leg[i].dq = 0;
            }
            low_cmd.motor_cmd_leg[i].kp = 200.0;
            low_cmd.motor_cmd_leg[i].kd = 20;
            low_cmd.motor_cmd_leg[i].tau = 0;
            low_cmd.motor_cmd_leg[i].dq = 0;
        }
    }
}

void PositionController::updateToLieDown(const Matrix34d &joint_pos, LowCmd &low_cmd)
{
    if (!lie_flag_)
    {
        time_ = 0;
        setLieDown(joint_pos);
    }
    else
    {
        time_ += dt_;
        for (int i = 0; i < 12; i++)
        {
            if (time_ < period_)
            {
                low_cmd.motor_cmd_leg[i].q = lie_spline_[i].position(time_);
                low_cmd.motor_cmd_leg[i].dq = lie_spline_[i].velocity(time_);
            }
            else if (time_ >= period_)
            {
                low_cmd.motor_cmd_leg[i].q = lie_joint_pos_(i);
                low_cmd.motor_cmd_leg[i].dq = 0;
            }

            low_cmd.motor_cmd_leg[i].kp = 200.0;
            low_cmd.motor_cmd_leg[i].kd = 20;
            low_cmd.motor_cmd_leg[i].tau = 0;
            low_cmd.motor_cmd_leg[i].dq = 0;
        }
    }
}

void PositionController::updateArmToDefault(const VectorXd &joint_pos, LowCmd &low_cmd)
{
    if (!arm_flag_)
    {
        time_ = 0;
        setArmDefault(joint_pos);
    }

    for (int i = 0; i < joint_pos.size(); i++)
    {
        if (time_ < period_)
        {
            low_cmd.motor_cmd_arm[i].q = arm_spline_[i].position(time_);
            low_cmd.motor_cmd_arm[i].dq = arm_spline_[i].velocity(time_);
        }
        else if (time_ >= period_)
        {
            low_cmd.motor_cmd_arm[i].q = default_arm_joint_pos_(i);
            low_cmd.motor_cmd_arm[i].dq = 0;
        }

        low_cmd.motor_cmd_arm[i].kp = 100;
        low_cmd.motor_cmd_arm[i].kd = 0.01;
        low_cmd.motor_cmd_arm[i].tau = 0;
        low_cmd.motor_cmd_arm[i].dq = 0;
        std::cout << low_cmd.motor_cmd_arm[i].dq << "  ";
    }
    std::cout << std::endl;
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

void PositionController::setArmDefault(const VectorXd &current_pos)
{
    arm_flag_ = true;
    CubicSpline::SplineNode start, end;
    for (int i = 0; i < current_pos.size(); i++)
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
