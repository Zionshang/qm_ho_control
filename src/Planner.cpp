#include "Planner.h"

Planner::Planner(HighCmd *highCmd, Estimator *est, LowState *lowState, WholeBodyDynamics *wbDyn)
    : _highCmd(highCmd), _est(est), _lowState(lowState), _wbDyn(wbDyn)
{
    _gaitGen = new FootPlanner();

    _dt = _est->getTimeStep();
    _velBMax << 0.5, 0.5, 0.3;    // x,y,z direction
    _angVelBMax << 0.5, 0.5, 0.5; // x,y,z direction
    _velBMin = -_velBMax;
    _angVelBMin = -_angVelBMax;
    _velBCmd.setZero();
    _angVelBCmd.setZero();

    _velGMax << 0.5, 0.5, 0.5;    // x,y,z direction
    _angVelGMax << 0.5, 0.5, 0.5; // x,y,z direction
    _velGMin = -_velGMax;
    _angVelGMin = -_angVelGMax;
    _velGCmd.setZero();
    _angVelGCmd.setZero();

    // clang-format off
    _qAJMin  << -2.61,    0, -2.87, -1.51, -1.34, -2.79;
    _qAJMax  <<  2.61, 2.96,     0,  1.51,  1.34,  2.79;
    _dqAJMax <<   0.5,  0.5,   0.5,   0.5,   0.5,   0.5;
    // clang-format on
    _dqAJMax = 3 * _dqAJMax.eval();
    _dqAJMin = -_dqAJMax;
    _dqAJCmd.setZero();
}

Planner::~Planner()
{
    delete _gaitGen;
};

void Planner::setDesiredTraj(const GaitState &gait_state)
{
    _rotB_d = quat2RotMat(_highCmd->quatB);
    if (_lastWorkMode != WorkMode::ARM_JOINT && _est->getWorkMode() == WorkMode::ARM_JOINT)
        _isModeChange = true;
    else
        _isModeChange = false;

    bodyPlan();
    gripperPlan();
    armJointPlan();

    _gaitGen->update(_est->body_state(), gait_state, _est->getPosF(), _highCmd->velB, _highCmd->angVelB, _highCmd->posF, _highCmd->velF); // foot trajectory
    _lastWorkMode = _est->getWorkMode();
}

void Planner::showDemo(const GaitState &gait_state)
{
    double tNow = _est->getCurrentTime();

    // body plan
    _velBCmd.setZero();
    _angVelBCmd.setZero();

    // double accB = 0.2, vleXMax = 0.5;
    // double accAngB = 0.2, velAngZMax = 0.3;
    // _velBCmd(0) = accB * tNow > vleXMax ? vleXMax : (accB * tNow);
    // _angVelBCmd(2) = accAngB * tNow > velAngZMax ? velAngZMax : (accAngB * tNow);

    _rotB_d = quat2RotMat(_highCmd->quatB);
    _highCmd->velB = _rotB_d * _velBCmd;
    _highCmd->angVelB = _rotB_d * _angVelBCmd;

    _highCmd->posB += _highCmd->velB * _dt;
    _highCmd->quatB += quatDeriv(_highCmd->quatB, _highCmd->angVelB) * _dt;
    _highCmd->quatB /= _highCmd->quatB.norm();

    // arm joint plan
    // _highCmd->qArm.setZero();
    // _highCmd->dqArm.setZero();
    Vec6 qArm_d0, qArm_d1, qArm_d2, qArm_d3, qArm_d4;
    double t1, t2, t3, t4, tTemp;
    qArm_d0.setZero();
    qArm_d1 << 0, 3, -2.5, 0.5, 0, 0;
    qArm_d2 << -3.14 / 2, 3, -2.5, 1, 0, 0;
    qArm_d3 << 3.14 / 2, 3, -2.5, 1, 0, 0;
    qArm_d4.setZero();
    tTemp = 0.5;
    t1 = 0.8, t2 = t1 + 0.8 + tTemp, t3 = t2 + 1.2 + tTemp, t4 = t3 + 1.2 + tTemp;
    if (tNow < t1)
        cubicSpline(qArm_d0, qArm_d1, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, t1, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    else if (tNow < t1 + tTemp)
    {
        _highCmd->qAJ = qArm_d1;
        _highCmd->dqAJ.setZero();
    }
    else if (tNow < t2)
        cubicSpline(qArm_d1, qArm_d2, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), t1 + tTemp, t2, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    else if (tNow < t2 + tTemp)
    {
        _highCmd->qAJ = qArm_d2;
        _highCmd->dqAJ.setZero();
    }
    else if (tNow < t3)
        cubicSpline(qArm_d2, qArm_d3, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), t2 + tTemp, t3, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    else if (tNow < t3 + tTemp)
    {
        _highCmd->qAJ = qArm_d3;
        _highCmd->dqAJ.setZero();
    }
    // else if (tNow < t4)
    //     cubicSpline(qArm_d3, qArm_d4, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), t3 + tTemp, t4, tNow, _highCmd->qArm, _highCmd->dqArm);

    // com plan
    comPlan();
    _gaitGen->update(_est->body_state(), gait_state, _est->getPosF(), _highCmd->velB, _highCmd->angVelB, _highCmd->posF, _highCmd->velF); // foot trajectory
}

void Planner::showFrontMaxJointVelDemo(const GaitState &gait_state)
{
    double tNow = _est->getCurrentTime();

    // body plan
    _velBCmd.setZero();
    _angVelBCmd.setZero();

    _rotB_d = quat2RotMat(_highCmd->quatB);
    _highCmd->velB = _rotB_d * _velBCmd;
    _highCmd->angVelB = _rotB_d * _angVelBCmd;

    _highCmd->posB += _highCmd->velB * _dt;
    _highCmd->quatB += quatDeriv(_highCmd->quatB, _highCmd->angVelB) * _dt;
    _highCmd->quatB /= _highCmd->quatB.norm();

    // arm joint plan
    // _highCmd->qArm.setZero();
    // _highCmd->dqArm.setZero();
    Vec6 qArm_d0, qArm_d1;
    double t1;
    qArm_d0.setZero();
    qArm_d1 << 0, 3.2, -2.6, 0.9, 0, 0;
    t1 = 0.3;
    if (tNow < t1)
        cubicSpline(qArm_d0, qArm_d1, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, t1, tNow, _highCmd->qAJ, _highCmd->dqAJ);

    // com plan
    comPlan();
    _gaitGen->update(_est->body_state(), gait_state, _est->getPosF(), _highCmd->velB, _highCmd->angVelB, _highCmd->posF, _highCmd->velF); // foot trajectory
}

void Planner::showPickingDemo(const GaitState &gait_state)
{
    double tNow = _est->getCurrentTime();

    double tForward, tTrun, tSideway, tFinal;
    tForward = 5, tTrun = tForward + 5, tSideway = tTrun + 8, tFinal = tSideway + 5;

    Vec3 rpyForward, rpyTurn, rpySideway, rpyFinal, rpyNow;
    rpyForward << 0, 15 * M_PI / 180, 0;
    rpyTurn << 0, 0, M_PI;
    rpySideway << -5 * M_PI / 180, 15 * M_PI / 180, M_PI;
    rpyFinal << 0, 0, M_PI;

    Vec3 posForward, posTurn, posSideway, posFinal, posInitial;
    posForward << 1.5, 0, 0.42;
    posTurn << 1.5, 0, 0.42;
    posSideway << 0.75, 0, 0.35;
    posFinal << 0, 0, 0.42;
    posInitial << 0, 0, 0.42;

    Vec6 qArmForward, qArmTurn, qArmSideway, qArmFinal;
    qArmForward << 0, 3, -2.5, 0.5, 0, 0;
    qArmTurn.setZero();
    qArmSideway << 3.14 / 2, 2.45, -1.68, 0.5, 0, 0;
    qArmFinal.setZero();

    if (tNow < tForward)
    {
        cubicSpline(Eigen::MatrixXd::Zero(3, 1), rpyForward, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, tForward, tNow, rpyNow, _angVelBCmd);
        cubicSpline(posInitial, posForward, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, tForward, tNow, _highCmd->posB, _highCmd->velB);
        cubicSpline(Eigen::MatrixXd::Zero(6, 1), qArmForward, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, tForward, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    }
    else if (tNow < tTrun)
    {
        cubicSpline(rpyForward, rpyTurn, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1), tForward, tTrun, tNow, rpyNow, _angVelBCmd);
        cubicSpline(posForward, posTurn, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(6, 1), tForward, tTrun, tNow, _highCmd->posB, _highCmd->velB);
        cubicSpline(qArmForward, qArmTurn, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), tForward, tTrun, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    }
    else if (tNow < tSideway)
    {
        cubicSpline(rpyTurn, rpySideway, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1), tTrun, tSideway, tNow, rpyNow, _angVelBCmd);
        cubicSpline(posTurn, posSideway, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(6, 1), tTrun, tSideway, tNow, _highCmd->posB, _highCmd->velB);
        cubicSpline(qArmTurn, qArmSideway, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), tTrun, tSideway, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    }
    else if (tNow < tFinal)
    {
        cubicSpline(rpySideway, rpyFinal, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1), tSideway, tFinal, tNow, rpyNow, _angVelBCmd);
        cubicSpline(posSideway, posFinal, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(6, 1), tSideway, tFinal, tNow, _highCmd->posB, _highCmd->velB);
        cubicSpline(qArmSideway, qArmFinal, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), tSideway, tFinal, tNow, _highCmd->qAJ, _highCmd->dqAJ);
    }
    else
    {
        _highCmd->posB = posFinal;
        _highCmd->velB.setZero();
        rpyNow = rpyFinal;
        _angVelBCmd.setZero();
        _highCmd->qAJ = qArmFinal;
        _highCmd->dqAJ.setZero();
    }

    _highCmd->quatB = rpy2Quat(rpyNow);
    _rotB_d = quat2RotMat(_highCmd->quatB);
    _highCmd->angVelB = _rotB_d * _angVelBCmd;

    // arm joint plan
    // _highCmd->qArm.setZero();
    // _highCmd->dqArm.setZero();

    // com plan
    comPlan();
    _gaitGen->update(_est->body_state(), gait_state, _est->getPosF(), _highCmd->velB, _highCmd->angVelB, _highCmd->posF, _highCmd->velF); // foot trajectory
}

void Planner::bodyPlan()
{
    // get commond, expressed in BODY frame
    if (_est->getWorkMode() == WorkMode::ARM_FIXED_BODY || _est->getWorkMode() == WorkMode::ARM_FIXED_WORLD)
    {
        _velBCmd(0) = invNormalize(_lowState->userValue.lx, _velBMin(0), _velBMax(0)); // x
        _velBCmd(1) = invNormalize(_lowState->userValue.ly, _velBMin(1), _velBMax(1)); // y
        _velBCmd(2) = invNormalize(_lowState->userValue.z, _velBMin(2), _velBMax(2));  // z

        _angVelBCmd(0) = 0;                                                                     // wx
        _angVelBCmd(1) = invNormalize(_lowState->userValue.rx, _angVelBMin(1), _angVelBMax(1)); // wy
        _angVelBCmd(2) = invNormalize(_lowState->userValue.ry, _angVelBMin(2), _angVelBMax(2)); // wz
    }
    else
    {
        _velBCmd.setZero();
        _angVelBCmd.setZero();
    }

    // velocity transformation from BODY to WORLD
    _highCmd->velB = _rotB_d * _velBCmd;
    _highCmd->angVelB = _rotB_d * _angVelBCmd;

    // position iteration
    _highCmd->posB += _highCmd->velB * _dt;
    _highCmd->quatB += quatDeriv(_highCmd->quatB, _highCmd->angVelB) * _dt;
    _highCmd->quatB /= _highCmd->quatB.norm();
}

void Planner::showSideMaxJointVelDemo(const GaitState &gait_state)
{
    double tNow = _est->getCurrentTime();

    // body plan
    _velBCmd.setZero();
    _angVelBCmd.setZero();

    _rotB_d = quat2RotMat(_highCmd->quatB);
    _highCmd->velB = _rotB_d * _velBCmd;
    _highCmd->angVelB = _rotB_d * _angVelBCmd;

    _highCmd->posB += _highCmd->velB * _dt;
    _highCmd->quatB += quatDeriv(_highCmd->quatB, _highCmd->angVelB) * _dt;
    _highCmd->quatB /= _highCmd->quatB.norm();

    // arm joint plan
    // _highCmd->qArm.setZero();
    // _highCmd->dqArm.setZero();
    Vec6 qArm_d0, qArm_d1;
    double t1;
    qArm_d0.setZero();
    qArm_d1 << -M_PI / 2, 3.2, -2.6, 0.9, 0, 0;
    t1 = 2.9;
    if (tNow < t1)
        cubicSpline(qArm_d0, qArm_d1, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), 0.0, t1, tNow, _highCmd->qAJ, _highCmd->dqAJ);

    // com plan
    comPlan();
    _gaitGen->update(_est->body_state(), gait_state, _est->getPosF(), _highCmd->velB, _highCmd->angVelB, _highCmd->posF, _highCmd->velF); // foot trajectory
}

void Planner::comPlan()
{
    _posGenCmd << _highCmd->posB, _highCmd->quatB, Eigen::MatrixXd::Zero(12, 1), _highCmd->qAJ;
    _velGenCmd << _highCmd->velB, _highCmd->angVelB, Eigen::MatrixXd::Zero(12, 1), _highCmd->dqAJ;
    _wbDyn->setCoMPosVel(_posGenCmd, _velGenCmd, _highCmd->posCoM, _highCmd->velCoM);
}

void Planner::gripperPlan()
{
    if (_est->getWorkMode() == WorkMode::ARM_CARTESIAN_BODY) // Control the gripper in BODY frame
    {
        _velGCmd(0) = invNormalize(_lowState->userValue.lx, _velGMin(0), _velGMax(0));          // x
        _velGCmd(1) = invNormalize(_lowState->userValue.ly, _velGMin(1), _velGMax(1));          // y
        _velGCmd(2) = invNormalize(_lowState->userValue.z, _velGMin(2), _velGMax(2));           // z
        _angVelGCmd(0) = 0;                                                                     // wx
        _angVelGCmd(1) = invNormalize(_lowState->userValue.rx, _angVelGMin(1), _angVelGMax(1)); // wy
        _angVelGCmd(2) = invNormalize(_lowState->userValue.ry, _angVelGMin(2), _angVelGMax(2)); // wz
    }
    else if (_est->getWorkMode() == WorkMode::ARM_FIXED_BODY) // Control the gripper fixed in BODY frame
    {
        _velGCmd.setZero();
        _angVelGCmd.setZero();
    }
    else if (_est->getWorkMode() == WorkMode::ARM_FIXED_WORLD) // Control the gripper fixed in WORLD frame
    {
        _highCmd->velG.setZero();
        _highCmd->angVelG.setZero();
    }
    else
    {
        _velGCmd.setZero();
        _angVelGCmd.setZero();
    }

    // velocity transformation from BODY to WORLD
    if (_est->getWorkMode() != WorkMode::ARM_FIXED_WORLD)
    {
        _highCmd->velG = _highCmd->velB + skew(_highCmd->angVelB) * (_est->getPosG() - _est->getPosB()) + _rotB_d * _velGCmd;
        _highCmd->angVelG = _rotB_d * _angVelGCmd;
    }
    // position iteration
    _highCmd->posG += _highCmd->velG * _dt;
    _highCmd->quatG += quatDeriv(_highCmd->quatG, _highCmd->angVelG) * _dt;
    _highCmd->quatG /= _highCmd->quatG.norm();
}

void Planner::armJointPlan()
{
    if (_isModeChange)
        _highCmd->qAJ = _est->getQArm();

    if (_est->getWorkMode() == WorkMode::ARM_JOINT) // Control the arm joint
    {
        _highCmd->dqAJ(0) = invNormalize(_lowState->userValue.ly, _dqAJMin(0), _dqAJMax(0)); // joint 1
        _highCmd->dqAJ(1) = invNormalize(_lowState->userValue.lx, _dqAJMin(1), _dqAJMax(1)); // joint 2
        _highCmd->dqAJ(2) = invNormalize(-_lowState->userValue.z, _dqAJMin(2), _dqAJMax(2)); // joint 3
        _highCmd->dqAJ(3) = invNormalize(_lowState->userValue.rx, _dqAJMin(3), _dqAJMax(3)); // joint 4
        _highCmd->dqAJ(4) = 0;                                                               // joint 5
        _highCmd->dqAJ(5) = invNormalize(_lowState->userValue.ry, _dqAJMin(4), _dqAJMax(4)); // joint 6                                                                    // joint 6

        _highCmd->qAJ += _highCmd->dqAJ * _dt;

        for (int i = 0; i < 6; i++)
            _highCmd->qAJ(i) = saturation(_highCmd->qAJ(i), _qAJMin(i), _qAJMax(i));
    }
}

void Planner::printDesiredTraj()
{
    std::cout << std::setw(12) << std::left << "Desired Body Position:     \t" << std::fixed << std::setprecision(2)
              << _highCmd->posB.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Desired Body Velocity:     \t" << std::fixed << std::setprecision(2)
              << _highCmd->velB.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Desired Gripper Position:  \t" << std::fixed << std::setprecision(2)
              << _highCmd->posG.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Desired Gripper Velocity:  \t" << std::fixed << std::setprecision(2)
              << _highCmd->velG.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Desired Arm Joint position:\t" << std::fixed << std::setprecision(2)
    //           << _highCmd->posArmJoint.transpose() << std::endl;
}
