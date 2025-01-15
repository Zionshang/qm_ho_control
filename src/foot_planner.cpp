#include "foot_planner.hpp"

FootPlanner::FootPlanner()
{
    height_swing = 0.04;
    offset_foot_ = 0.026;
    first_run_ = true;

    // used in caculation of foothold
    kx_ = 0.005;
    ky_ = 0.005;
    kyaw_ = 0.005;

    Vec24 posStill;                               // XY position when stance in still
    posStill << 0.2407, -0.2407, 0.2407, -0.2407, // clang-format off
                -0.138,  -0.138,  0.138,   0.138;  // clang-foramt on
    for (int i = 0; i < 4; ++i)
    {
        feet_radius_(i) = sqrt(pow(posStill(0, i), 2) + pow(posStill(1, i), 2));
        feet_init_angle(i) = atan2(posStill(1, i), posStill(0, i));
    }
}


void FootPlanner::update(const BodyState &body_state, const GaitState &gait_state, const Matrix34d &pos_feet,
                         const Vector3d &vel_body_ref, const Vector3d &angvel_body_ref,
                         Vec34 &pos_feet_ref, Vec34 &vel_feet_ref)
{
    phase_ =  gait_state.phase;
    period_swing_ = gait_state.period_swing;
    period_stance_ = gait_state.period_stance;

    if (first_run_)
    {
        pos_start_ = pos_feet;
        first_run_ = false;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (gait_state.contact(i) == 1)
        {
            if (phase_(i) < 0.5)
                pos_start_.col(i) = pos_feet.col(i);
            pos_feet_ref.col(i) = pos_start_.col(i);
            vel_feet_ref.col(i).setZero();
        }
        else
        {
            pos_end_.col(i) = calcFootholdPosition(body_state, vel_body_ref, angvel_body_ref, i);
            pos_feet_ref.col(i) = calcReferenceFootPosition(i);
            vel_feet_ref.col(i) = calcReferenceFootVelocity(i);
        }
    }
}

Vec3 FootPlanner::calcFootholdPosition(const BodyState &body_state, const Vector3d &vel_body_ref,
                                       const Vector3d &angvel_body_ref, int leg_id)
{
    const Vec3 pos_body = body_state.pos;
    const Vec3 vel_body = body_state.vel;
    const RotMat rotmat_body = body_state.rotmat;
    const Vec3 angvel_body = body_state.angvel;

    // TODO: 是否需要改成相对于body系下
    // Translation in x,y axis
    next_step_(0) = vel_body(0) * (1 - phase_(leg_id)) * period_swing_ + vel_body(0) * period_stance_ / 2 + kx_ * (vel_body(0) - vel_body_ref_(0));
    next_step_(1) = vel_body(1) * (1 - phase_(leg_id)) * period_swing_ + vel_body(1) * period_stance_ / 2 + ky_ * (vel_body(1) - vel_body_ref_(1));
    next_step_(2) = 0; 

    // rotation about z axis
    yaw_ = rotMat2RPY(rotmat_body)(2);
    next_yaw_ = angvel_body(2) * (1 - phase_(leg_id)) * period_swing_ + angvel_body(2) * period_stance_ / 2 + kyaw_ * (angvel_body_ref_(2) - angvel_body(2));
    next_step_(0) += feet_radius_(leg_id) * cos(yaw_ + feet_init_angle(leg_id) + next_yaw_);
    next_step_(1) += feet_radius_(leg_id) * sin(yaw_ + feet_init_angle(leg_id) + next_yaw_);

    Vec3 footholdPos;
    footholdPos = pos_body + next_step_;
    footholdPos(2) = offset_foot_;

    return footholdPos;
}

Vec3 FootPlanner::calcReferenceFootPosition(int leg_id)
{
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(pos_start_.col(leg_id)(0), pos_end_.col(leg_id)(0), phase_(leg_id));
    footPos(1) = cycloidXYPosition(pos_start_.col(leg_id)(1), pos_end_.col(leg_id)(1), phase_(leg_id));
    footPos(2) = cycloidZPosition(pos_start_.col(leg_id)(2), height_swing + offset_foot_, phase_(leg_id));

    return footPos;
}

Vec3 FootPlanner::calcReferenceFootVelocity(int leg_id)
{
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(pos_start_.col(leg_id)(0), pos_end_.col(leg_id)(0), phase_(leg_id));
    footVel(1) = cycloidXYVelocity(pos_start_.col(leg_id)(1), pos_end_.col(leg_id)(1), phase_(leg_id));
    footVel(2) = cycloidZVelocity(height_swing + offset_foot_, phase_(leg_id));

    return footVel;
}

double FootPlanner::cycloidXYPosition(double start, double end, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
}

double FootPlanner::cycloidXYVelocity(double start, double end, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return (end - start) * (1 - cos(phasePI)) / period_swing_;
}

double FootPlanner::cycloidZPosition(double start, double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * (1 - cos(phasePI)) / 2 + start;
}

double FootPlanner::cycloidZVelocity(double h, double phase)
{
    double phasePI = 2 * M_PI * phase;
    return h * M_PI * sin(phasePI) / period_swing_;
}