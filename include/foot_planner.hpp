#pragma once
#include "gait_schedule.hpp"
#include "Estimator.h"

class FootPlanner
{
public:
    FootPlanner();
    void setGait(Vec3 vBd, Vec3 wBd);
    void update(const BodyState &body_state, const Vector4d &phase, const Vector4i &contact,
                const Matrix34d &pos_feet, double period_swing, double period_stance, 
                Vec34 &pos_feet_ref, Vec34 &vel_feet_ref);

private:
    Vector3d calcFootholdPosition(const BodyState &body_state, int leg_id);
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
    Vec2 vxyGoal_;
    double dYawGoal_, nextYaw_, yaw_;
    Vector4d feet_radius_, feet_init_angle;
    Vector3d next_step_;
};
