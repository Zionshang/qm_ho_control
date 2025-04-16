#include "common/low_state.hpp"
#include "common/low_cmd.hpp"
#include "ros2_interface.hpp"
#include "planner.hpp"
#include "controller.hpp"
#include "utils/logger.hpp"
#include "pinocchio_interface.hpp"
#include "kalman_filter_estimator.hpp"
#include "gait_schedule.hpp"
#include "ctrl_component.hpp"
#include "position_controller.hpp"

int main(int argc, char **argv)
{
        double timestep = 0.001; // in seconds

        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("qm_ho_control_node");

        shared_ptr pin_interface = std::make_shared<PinocchioInterface>();
        // shared_ptr webots_interface = std::make_shared<WebotsInterface>();
        shared_ptr ctrl_comp = std::make_shared<CtrlComponent>(pin_interface);
        shared_ptr gait_sche = std::make_shared<GaitSchedule>();
        shared_ptr planner = std::make_shared<Planner>(pin_interface, timestep);
        shared_ptr controller = std::make_shared<Controller>(pin_interface);
        shared_ptr position_controller = std::make_shared<PositionController>(timestep);
        shared_ptr estimator = std::make_shared<KalmanFilterEstimator>(pin_interface, timestep);
        shared_ptr ros2_interface = std::make_shared<Ros2Interface>(node);

        double current_time = 0.0;
        rclcpp::Rate rate(1.0 / timestep);

        while (rclcpp::ok())
        {
                rclcpp::spin_some(node);
                ros2_interface->recvState(ctrl_comp->low_state);
                ros2_interface->recvUserCmd(ctrl_comp->user_cmd);

                gait_sche->update(current_time, ctrl_comp->user_cmd.gait_name);
                estimator->update(ctrl_comp->low_state, gait_sche->contact(), ctrl_comp->robot_state);

                planner->update(ctrl_comp->user_cmd, ctrl_comp->robot_state,
                                gait_sche->gait_state(), ctrl_comp->target_robot_state);

                switch (ctrl_comp->user_cmd.ctrl_type)
                {

                case ControllerType::POSITION_STAND_UP:
                        position_controller->updateToStandUp(ctrl_comp->robot_state.joint.pos_leg,
                                                             ctrl_comp->low_cmd);
                        position_controller->updateArmToDefault(ctrl_comp->robot_state.joint.pos_arm,
                                                                ctrl_comp->low_cmd);
                        break;
                case ControllerType::POSITION_LIE_DOWN:
                        position_controller->updateToLieDown(ctrl_comp->robot_state.joint.pos_leg,
                                                             ctrl_comp->low_cmd);
                        position_controller->updateArmToDefault(ctrl_comp->robot_state.joint.pos_arm,
                                                                ctrl_comp->low_cmd);
                        break;
                case ControllerType::TORQUE_CONTROLLER:
                        controller->run(ctrl_comp->robot_state, ctrl_comp->target_robot_state,
                                        gait_sche->contact(), ctrl_comp->low_cmd);
                        break;
                case ControllerType::NONE:
                default:
                        break;
                }

                ros2_interface->sendCmd(ctrl_comp->low_cmd);
                current_time = current_time + timestep;
                // RCLCPP_INFO(node->get_logger(), "Current time: %.3f", current_time);

                rate.sleep();

        }
        rclcpp::shutdown();

        return 0;
}
