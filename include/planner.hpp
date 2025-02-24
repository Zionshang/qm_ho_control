#pragma once

#include "foot_planner.hpp"
#include "pinocchio_interface.hpp"

class Planner
{
public:
    Planner(PinocchioInterface *pin_interface);
    ~Planner();
    void update(const UserCommand &user_cmd, const RobotState &robot_state,
                const GaitState &gait_state, RobotState &robot_state_ref);

private:
    void bodyPlan(const UserCommand &user_cmd, BodyState &body_state_ref);
    void armJointPlan(JointState &joint_state_ref);
    void comPlan(RobotState &robot_state_ref, Vector3d &pos_com_ref, Vector3d &vel_com_ref);

    FootPlanner *foot_planner_;
    PinocchioInterface *pin_interface_;
    double dt_;
};
