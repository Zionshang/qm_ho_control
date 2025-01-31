#include "common/LowState.h"
#include "common/LowCmd.h"
#include "common/HighCmd.h"
#include "model/WholeBodyDynamics.h"
#include "IOWebots.h"
#include "Estimator.h"
#include "Planner.h"
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
        HighCmd *highCmd = new HighCmd();
        WholeBodyDynamics *wbDyn = new WholeBodyDynamics();
        PinocchioInterface *pin_interface = new PinocchioInterface();
        IOWebots *iowebots = new IOWebots(lowState, lowCmd);
        GaitSchedule *gait_sche = new GaitSchedule();
        Estimator *est = new Estimator(lowState, pin_interface);
        Planner *plan = new Planner(highCmd, est, lowState, wbDyn);
        Controller *ctlr = new Controller(est, highCmd, lowCmd, wbDyn);
        DataLog *log = new DataLog();
        KalmanFilterEstimator *kfe = new KalmanFilterEstimator(lowState, pin_interface, lowState->timeStep);
        CtrlComponent *ctrl_comp = new CtrlComponent(pin_interface);

        GaitName target_gait_name = GaitName::TROT;
        while (iowebots->isRunning())
        {
                iowebots->recvState();

                gait_sche->update(lowState->currentTime, target_gait_name);

                // // control
                // switch (lowState->getUserCmd())
                // {
                // case UserCommand::A:
                //     target_gait_name = GaitName::TROT;
                //     break;
                // case UserCommand::B:
                //     target_gait_name = GaitName::STANCE;
                //     break;
                // case UserCommand::X:
                //     target_gait_name = GaitName::RUNNING_TROT;
                //     break;
                // case UserCommand::Y:
                //     target_gait_name = GaitName::WALK;
                //     break;
                // }
                est->update(ctrl_comp->mutable_robot_state());
                // est->printState();
                // plan->setDesiredTraj();
                // kfe->update(gait_sche->getContact());
                // plan->showDemo();
                // plan->showFrontMaxJointVelDemo();
                plan->showPickingDemo(ctrl_comp->robot_state(), gait_sche->gait_state());
                // plan->showSideMaxJointVelDemo();
                // plan->printDesiredTraj();
                ctlr->run(ctrl_comp->robot_state(), gait_sche->contact());
                iowebots->sendCmd();
                log->loadData(est, highCmd);

#ifdef DESIREDTRAJ
                iowebots->drawDesiredTraj(highCmd);
#endif

                if (est->getCurrentTime() > 25)
                        break;
        }
        log->saveData();

        delete lowState;
        delete lowCmd;
        delete highCmd;
        delete iowebots;
        delete est;
        delete plan;
        delete ctlr;
        // delete trajDraw;
        // delete kfe;

        return 0;
}
