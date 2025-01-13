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

int main(int argc, char const *argv[])
{
    LowState *lowState = new LowState();
    LowCmd *lowCmd = new LowCmd();
    HighCmd *highCmd = new HighCmd();
    WholeBodyDynamics *wbDyn = new WholeBodyDynamics();
    PinocchioInterface *pin_interface = new PinocchioInterface();
    IOWebots *iowebots = new IOWebots(lowState, lowCmd);
    Estimator *est = new Estimator(GaitName::TROT, lowState, pin_interface);
    Planner *plan = new Planner(highCmd, est, lowState, wbDyn);
    Controller *ctlr = new Controller(est, highCmd, lowCmd, wbDyn);
    DataLog *log = new DataLog();

    while (iowebots->isRunning())
    {
        iowebots->recvState();
        est->setAllState();
        est->printState();
        // plan->setDesiredTraj();
        plan->showDemo();
        // plan->showFrontMaxJointVelDemo();
        // plan->showPickingDemo();
        // plan->showSideMaxJointVelDemo();
        // plan->printDesiredTraj();
        ctlr->run();
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

    return 0;
}