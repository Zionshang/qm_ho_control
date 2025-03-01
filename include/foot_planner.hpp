#pragma once
#include "gait_schedule.hpp"
#include "ctrl_component.hpp"
class FootPlanner
{
public:
    FootPlanner();
    // TODO: 更简洁的参数传递
    void update(const GaitState &gait_state, const BodyState &body_state, const FootState &feet_state,
                const BodyState &body_state_ref, FootState &feet_state_ref);

private:
    Vector3d calcFootholdPosition(const BodyState &body_state, const Vector3d &vel_body_ref,
                                  const Vector3d &angvel_body_ref, int leg_id);
    Vector3d calcReferenceFootPosition(int leg_id);
    Vector3d calcReferenceFootVelocity(int leg_id);
    double cycloidXYPosition(double start, double end, double phase);
    double cycloidXYVelocity(double start, double end, double phase);
    double cycloidZPosition(double startZ, double height, double phase);
    double cycloidZVelocity(double height, double phase);

    Matrix34d pos_start_, pos_end_;
    double height_swing;
    double offset_foot_; // Radius of sphere at foot
    bool first_run_;
    Vector4d phase_;
    double period_swing_;
    double period_stance_;

    // foothold
    double kx_, ky_, kyaw_;
    Vector3d vel_body_ref_;
    Vector3d angvel_body_ref_;
    double next_yaw_, yaw_;
    Vector4d feet_radius_, feet_init_angle;
    Vector3d next_step_;
};
