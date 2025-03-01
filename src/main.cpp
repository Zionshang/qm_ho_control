#include <memory>
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
        std::unique_ptr low_state = std::make_unique<LowState>();
        std::unique_ptr low_cmd = std::make_unique<LowCmd>();
        std::unique_ptr pin_interface = std::make_unique<PinocchioInterface>();
        std::unique_ptr webots_interface = std::make_unique<WebotsInterface>();
        std::unique_ptr gait_sche = std::make_unique<GaitSchedule>();
        std::unique_ptr plan = std::make_unique<Planner>(pin_interface.get());
        std::unique_ptr ctlr = std::make_unique<Controller>(pin_interface.get());
        std::unique_ptr kfe = std::make_unique<KalmanFilterEstimator>(low_state.get(), pin_interface.get(), timestep);
        std::unique_ptr ctrl_comp = std::make_unique<CtrlComponent>(pin_interface.get());

        while (webots_interface->isRunning())
        {
                webots_interface->recvState(*low_state);
                webots_interface->recvUserCmd(ctrl_comp->mutable_user_cmd());
                
                gait_sche->update(webots_interface->current_time(), ctrl_comp->user_cmd().gait_name);
                kfe->update(gait_sche->contact(), ctrl_comp->mutable_robot_state());

                plan->update(ctrl_comp->user_cmd(), ctrl_comp->robot_state(),
                gait_sche->gait_state(), ctrl_comp->mutable_target_robot_state());

                ctlr->run(ctrl_comp->robot_state(), ctrl_comp->target_robot_state(), gait_sche->contact(), *low_cmd);
                webots_interface->sendCmd(*low_cmd);
        }

        return 0;
}
