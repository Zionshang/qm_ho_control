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

int main()
{
        double timestep = 0.001; // in seconds

        shared_ptr pin_interface = std::make_shared<PinocchioInterface>();
        shared_ptr webots_interface = std::make_shared<WebotsInterface>();
        shared_ptr ctrl_comp = std::make_shared<CtrlComponent>(pin_interface);
        shared_ptr gait_sche = std::make_shared<GaitSchedule>();
        shared_ptr planner = std::make_shared<Planner>(pin_interface, timestep);
        shared_ptr controller = std::make_shared<Controller>(pin_interface);
        shared_ptr estimator = std::make_shared<KalmanFilterEstimator>(pin_interface, timestep);

        while (webots_interface->isRunning())
        {
                webots_interface->recvState(ctrl_comp->low_state);
                webots_interface->recvUserCmd(ctrl_comp->user_cmd);

                gait_sche->update(webots_interface->current_time(), ctrl_comp->user_cmd.gait_name);
                estimator->update(ctrl_comp->low_state, gait_sche->contact(), ctrl_comp->robot_state);

                planner->update(ctrl_comp->user_cmd, ctrl_comp->robot_state,
                                gait_sche->gait_state(), ctrl_comp->target_robot_state);

                controller->run(ctrl_comp->robot_state, ctrl_comp->target_robot_state,
                                gait_sche->contact(), ctrl_comp->low_cmd);

                webots_interface->sendCmd(ctrl_comp->low_cmd);
        }

        return 0;
}
