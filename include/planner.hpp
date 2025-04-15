#pragma once

#include "foot_planner.hpp"
#include "pinocchio_interface.hpp"

class Planner
{
public:
    Planner(shared_ptr<PinocchioInterface> pin_interface, double timestep);
    void update(const UserCommand &user_cmd, const RobotState &robot_state,
                const GaitState &gait_state, RobotState &robot_state_ref);

private:
    void bodyPlan(const UserCommand &user_cmd, BodyState &body_state_ref);
    void armJointPlan(const UserCommand &user_cmd, JointState &joint_state_ref);
    void comPlan(RobotState &robot_state_ref, Vector3d &pos_com_ref, Vector3d &vel_com_ref);

    shared_ptr<FootPlanner> foot_planner_;
    shared_ptr<PinocchioInterface> pin_interface_;
    double dt_;
};
