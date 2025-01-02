#include "Estimator.h"
#include <iostream>

Estimator::Estimator(GaitName gaitName, LowState *lowState, QuadrupedArmRobot *robModel, WholeBodyDynamics *wbDyn)
    : _lowState(lowState), _robModel(robModel), _wbDyn(wbDyn)
{
    _gaitSche = new GaitSchedule(gaitName);
}

Estimator::~Estimator()
{
    delete _gaitSche;
}

void Estimator::setAllState()
{
    // joint
    _qLeg = _lowState->getQLeg();
    _dqLeg = _lowState->getDqLeg();

    // body
    _posB << _lowState->supervisor.robotPos[0], _lowState->supervisor.robotPos[1], _lowState->supervisor.robotPos[2];
    _velB << _lowState->supervisor.robotVel[0], _lowState->supervisor.robotVel[1], _lowState->supervisor.robotVel[2];
    _quatB = _lowState->getQuaternion();
    _rotB = quat2RotMat(_quatB);
    _angVelB << _rotB * _lowState->getGyro();

    // CoM
    _posGen << _posB, _quatB, vec34ToVec12(_qLeg), _qArm;
    _velGen << _rotB.transpose() * _velB, _rotB.transpose() * _angVelB, vec34ToVec12(_dqLeg), _dqArm;
    _wbDyn->setCoMPosVel(_posGen, _velGen, _posCoM, _velCoM);

    // foot
    for (int i = 0; i < 4; i++)
    {
        _posF.col(i) = _posB + _rotB * _robModel->calFootPos(_qLeg.col(i), i);
        _velF.col(i) = _velB + _rotB * (skew(_lowState->getGyro()) * _robModel->calFootPos(_qLeg.col(i), i) + _robModel->calFootVel(_qLeg.col(i), _dqLeg.col(i), i));
    }

    // gait
    _gaitSche->run(_lowState->currentTime);
    _phase = _gaitSche->getPhase();
    _contact = _gaitSche->getContact();

    // arm
    _qArm = _lowState->getQArm();
    _dqArm = _lowState->getDqArm();
    _robModel->setAllGriperState(_qArm, _posG2B_B, _quatG2B, _dqArm, _velG2B_B, _angVelG2B_B);
    _posG = _posB + _rotB * _posG2B_B;
    _quatG = quatTimes(_quatB, _quatG2B);
    _rotG = quat2RotMat(_quatG);
    _velG = _velB + _rotB * (skew(_lowState->getGyro()) * _posG2B_B + _velG2B_B);
    _angVelG = _angVelB + _rotB * _angVelG2B_B;

    // control
    switch (_lowState->getUserCmd())
    {
    case UserCommand::A:
        _workMode = WorkMode::ARM_CARTESIAN_BODY;
        break;
    case UserCommand::B:
        _workMode = WorkMode::ARM_JOINT;
        break;
    case UserCommand::X:
        _workMode = WorkMode::ARM_FIXED_BODY;
        break;
    case UserCommand::Y:
        _workMode = WorkMode::ARM_FIXED_WORLD;
        break;
    }
}

void Estimator::printState()
{
    // std::cout << std::setw(12) << std::left << "Current Body Position:     \t" << std::fixed << std::setprecision(2)
    //           << _posB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Body Velocity:     \t" << std::fixed << std::setprecision(2)
    //           << _velB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Position:  \t" << std::fixed << std::setprecision(2)
    //           << _posB.transpose() << std::endl;
    // std::cout << std::setw(12) << std::left << "Current Gripper Velocity:  \t" << std::fixed << std::setprecision(2)
    //           << _velB.transpose() << std::endl;
    std::cout << std::setw(12) << std::left << "Current Arm Joint position:\t" << std::fixed << std::setprecision(2)
              << _qArm.transpose() << std::endl;
}
