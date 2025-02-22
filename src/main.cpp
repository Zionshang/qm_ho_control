#include "common/LowState.h"
#include "common/LowCmd.h"
#include "model/WholeBodyDynamics.h"
#include "IOWebots.h"
#include "Estimator.h"
#include "planner.hpp"
#include "Controller.h"
#include "DataLog.h"
#include "pinocchio_interface.hpp"
#include "kalman_filter_estimator.hpp"
#include "gait_schedule.hpp"
#include "ctrl_component.hpp"

int main()
{
        LowState *lowState = new LowState();
        LowCmd *lowCmd = new LowCmd();
        WholeBodyDynamics *wbDyn = new WholeBodyDynamics();
        PinocchioInterface *pin_interface = new PinocchioInterface();
        IOWebots *iowebots = new IOWebots(lowState, lowCmd);
        GaitSchedule *gait_sche = new GaitSchedule();
        Estimator *est = new Estimator(lowState, pin_interface);
        Planner *plan = new Planner(pin_interface);
        Controller *ctlr = new Controller(est, lowCmd, wbDyn);
        DataLog *log = new DataLog();
        KalmanFilterEstimator *kfe = new KalmanFilterEstimator(lowState, pin_interface, lowState->timeStep);
        CtrlComponent *ctrl_comp = new CtrlComponent(pin_interface);
        while (iowebots->isRunning())
        {
                iowebots->recvState();
                iowebots->recvUserCmd(ctrl_comp->mutable_user_cmd());
                gait_sche->update(lowState->currentTime, ctrl_comp->user_cmd().gait_name);
                est->update(ctrl_comp->mutable_robot_state());
                // est->printState();
                plan->update(ctrl_comp->user_cmd(), ctrl_comp->robot_state(),
                                     gait_sche->gait_state(), ctrl_comp->mutable_target_robot_state());
                // kfe->update(gait_sche->getContact());

                ctlr->run(ctrl_comp->robot_state(), ctrl_comp->target_robot_state(), gait_sche->contact());
                iowebots->sendCmd();
                log->loadData(est);
        }
        log->saveData();

        delete lowState;
        delete lowCmd;
        delete iowebots;
        delete est;
        delete plan;
        delete ctlr;
        // delete trajDraw;
        // delete kfe;

        return 0;
}
