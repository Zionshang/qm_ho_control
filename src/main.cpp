#include "common/low_state.hpp"
#include "common/low_cmd.hpp"
#include "webots_interface.hpp"
#include "Estimator.h"
#include "planner.hpp"
#include "controller.hpp"
#include "DataLog.h"
#include "pinocchio_interface.hpp"
#include "kalman_filter_estimator.hpp"
#include "gait_schedule.hpp"
#include "ctrl_component.hpp"

int main()
{
        double timestep = 0.001; // in seconds
        LowState *low_state = new LowState();
        LowCmd *low_cmd = new LowCmd();
        PinocchioInterface *pin_interface = new PinocchioInterface();
        WebotsInterface *webots_interface = new WebotsInterface();
        GaitSchedule *gait_sche = new GaitSchedule();
        Estimator *est = new Estimator(low_state, pin_interface);
        Planner *plan = new Planner(pin_interface);
        Controller *ctlr = new Controller(est, pin_interface);
        DataLog *log = new DataLog();
        KalmanFilterEstimator *kfe = new KalmanFilterEstimator(low_state, pin_interface, timestep);
        CtrlComponent *ctrl_comp = new CtrlComponent(pin_interface);
        while (webots_interface->isRunning())
        {
                webots_interface->recvState(*low_state);
                webots_interface->recvUserCmd(ctrl_comp->mutable_user_cmd());
                gait_sche->update(webots_interface->current_time(), ctrl_comp->user_cmd().gait_name);
                est->update(ctrl_comp->mutable_robot_state());
                // est->printState();
                plan->update(ctrl_comp->user_cmd(), ctrl_comp->robot_state(),
                             gait_sche->gait_state(), ctrl_comp->mutable_target_robot_state());
                // kfe->update(gait_sche->getContact());

                ctlr->run(ctrl_comp->robot_state(), ctrl_comp->target_robot_state(), gait_sche->contact(), *low_cmd);
                webots_interface->sendCmd(*low_cmd);
                log->loadData(est);
        }
        log->saveData();

        delete low_state;
        delete low_cmd;
        delete est;
        delete plan;
        delete ctlr;
        // delete trajDraw;
        // delete kfe;

        return 0;
}
