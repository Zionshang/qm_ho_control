#include "common/low_state.hpp"
#include "common/low_cmd.hpp"
#include "webots_interface.hpp"
#include "planner.hpp"
#include "controller.hpp"
#include "utils/logger.hpp"
#include "pinocchio_interface.hpp"
#include "kalman_filter_estimator.hpp"
#include "gait_schedule.hpp"
#include "ctrl_component.hpp"
#include "position_controller.hpp"
#include "cheat_estimator.hpp"
#include "kinematics_mpc.hpp"
#include "interpolator.hpp"

int main()
{
        double timestep = 0.001; // in seconds

        shared_ptr pin_interface = std::make_shared<PinocchioInterface>();
        shared_ptr webots_interface = std::make_shared<WebotsInterface>();
        shared_ptr ctrl_comp = std::make_shared<CtrlComponent>(pin_interface);
        shared_ptr gait_sche = std::make_shared<GaitSchedule>();
        shared_ptr planner = std::make_shared<Planner>(pin_interface, timestep);
        shared_ptr controller = std::make_shared<Controller>(pin_interface);
        shared_ptr position_controller = std::make_shared<PositionController>(timestep);
        shared_ptr estimator = std::make_shared<KalmanFilterEstimator>(pin_interface, timestep);
        shared_ptr cheat_estimator = std::make_shared<CheatEstimator>(pin_interface);
        shared_ptr kmpc = std::make_shared<KinematicsMPC>();
        shared_ptr interpolator = std::make_shared<Interpolator>(kmpc->model());

        int itr = 0;
        int frequence_kinematics_mpc = 100;
        const double dt = 0.001; // Time step for integration
        std::vector<VectorXd> x_kmpc;

        while (webots_interface->isRunning())
        {
                if (itr % frequence_kinematics_mpc == 0)
                {

                        Eigen::VectorXd x(kmpc->nq() + kmpc->nv()); // 状态量大小为 7 + 6 + 6 + 6 = 25

                        const auto &robot_state = ctrl_comp->robot_state;
                        x << robot_state.body.pos, robot_state.body.quat.coeffs(), robot_state.joint.pos_arm,
                            robot_state.body.vel, robot_state.body.angvel, robot_state.joint.vel_arm;

                        Eigen::Vector3d target_position(1.5, 0.0, 0.5);            // 目标位置
                        Eigen::Quaterniond target_orientation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())); // 绕y轴旋转90度
                        // Eigen::Quaterniond target_orientation(1.0, 0.0, 0.0, 0.0); // 目标方向（单位四元数）

                        kmpc->setTarget(x, target_position, target_orientation);
                        kmpc->solve(x);
                        itr = 0;
                        x_kmpc = kmpc->getSolution();

                        // std::cout << "x_kmpc:" << std::endl;
                        // for (size_t i = 0; i < x_kmpc.size(); ++i)
                        // {
                        //         std::cout << "Step " << i << ": " << x_kmpc[i].transpose() << std::endl;
                        // }
                }

                double delay = itr * dt;
                VectorXd x_interp(kmpc->nq() + kmpc->nv());
                interpolator->interpolateState(delay, kmpc->dt(), x_kmpc, x_interp);
                // std::cout << "x_interp: " << x_interp.transpose() << std::endl;

                auto &target_robot_state = ctrl_comp->target_robot_state;

                target_robot_state.body.pos = x_interp.head(3); // 机身位置
                target_robot_state.body.quat = Eigen::Quaterniond(
                    x_interp(6), x_interp(3), x_interp(4), x_interp(5));    // 四元数 (w, x, y, z)
                target_robot_state.joint.pos_arm = x_interp.segment(7, 5);  // 机械臂关节位置
                target_robot_state.body.vel = x_interp.segment(12, 3);      // 机身线速度
                target_robot_state.body.angvel = x_interp.segment(15, 3);   // 机身角速度
                target_robot_state.joint.vel_arm = x_interp.segment(18, 5); // 机械臂关节速度

                webots_interface->recvState(ctrl_comp->low_state);
                webots_interface->recvUserCmd(ctrl_comp->user_cmd);

                gait_sche->update(webots_interface->current_time(), ctrl_comp->user_cmd.gait_name);
                // estimator->update(ctrl_comp->low_state, gait_sche->contact(), ctrl_comp->robot_state);
                cheat_estimator->update(webots_interface->CheatNode(), ctrl_comp->low_state, ctrl_comp->robot_state);

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
                // controller->run(ctrl_comp->robot_state, ctrl_comp->target_robot_state,
                //                 gait_sche->contact(), ctrl_comp->low_cmd);

                webots_interface->sendCmd(ctrl_comp->low_cmd);

                itr++;
        }

        return 0;
}
