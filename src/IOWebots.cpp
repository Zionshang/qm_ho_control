#include "IOWebots.h"

IOWebots::IOWebots(LowState *lowState, LowCmd *lowCmd)
    : _lowState(lowState), _lowCmd(lowCmd)
{
    _supervisor = new webots::Supervisor();
    _timeStep = (int)_supervisor->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << _timeStep << std::endl;
    initRecv();
    initSend();

    _lastqLeg << 0.0, 0.0, 0.0, 0.0,
        0.67, 0.67, 0.67, 0.67,
        -1.3, -1.3, -1.3, -1.3;
    _lastqArm << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

IOWebots::~IOWebots()
{
    delete _supervisor;
}

void IOWebots::recvState()
{
    // time
    _lowState->currentTime = _supervisor->getTime(); // second

    // sensor
    const double *imuData = _imu->getQuaternion(); // x,y,z,w
    const double *gyroData = _gyro->getValues();
    const double *accelerometerData = _accelerometer->getValues();
    const double *robotPosData = _robotNode->getPosition();
    const double *robotVelData = _robotNode->getVelocity();

    for (int i = 0; i < 3; i++)
    {
        _lowState->imu.quaternion[i] = static_cast<double>(imuData[i]);
        _lowState->imu.gyro[i] = static_cast<double>(gyroData[i]);
        _lowState->imu.accelerometer[i] = static_cast<double>(accelerometerData[i]);
        _lowState->supervisor.robotPos[i] = static_cast<double>(robotPosData[i]);
        _lowState->supervisor.robotVel[i] = static_cast<double>(robotVelData[i]);
    }
    _lowState->imu.quaternion[3] = static_cast<double>(imuData[3]);

    for (int i = 0; i < 12; i++)
    {
        _lowState->motorLeg[i].q = _jointSensorLeg[i]->getValue();
        _lowState->motorLeg[i].dq = (_lowState->motorLeg[i].q - _lastqLeg(i)) / double(_timeStep) * 1000;
        _lastqLeg(i) = _lowState->motorLeg[i].q;
    }

    for (int i = 0; i < 6; i++)
    {
        _lowState->motorArm[i].q = _jointSensorArm[i]->getValue();
        _lowState->motorArm[i].dq = (_lowState->motorArm[i].q - _lastqArm(i)) / double(_timeStep) * 1000;
        _lastqArm(i) = _lowState->motorArm[i].q;
    }

    // joystick cmd
    switch (_joystick->getPressedButton())
    {
    case 0:
        _lowState->userCmd = UserCommand::A;
        std::cout << "You pressed A\n";
        break;
    case 1:
        _lowState->userCmd = UserCommand::B;
        std::cout << "You pressed B\n";
        break;
    case 3:
        _lowState->userCmd = UserCommand::X;
        std::cout << "You pressed X\n";
        break;
    case 4:
        _lowState->userCmd = UserCommand::Y;
        std::cout << "You pressed Y\n";
        break;
    case 8:
        _lowState->userValue.z = 1.0;
        break;
    case 9:
        _lowState->userValue.z = -1.0;
        break;
    default:
        _lowState->userValue.z = 0;
        break;
    }

    _lowState->userValue.ly = -killSmallOffset(double(_joystick->getAxisValue(0)) / 32767, 0.25);
    _lowState->userValue.lx = -killSmallOffset(double(_joystick->getAxisValue(1)) / 32767, 0.25);
    _lowState->userValue.ry = -killSmallOffset(double(_joystick->getAxisValue(2)) / 32767, 0.25);
    _lowState->userValue.rx = -killSmallOffset(double(_joystick->getAxisValue(3)) / 32767, 0.25);
}

void IOWebots::sendCmd()
{
    for (int i = 0; i < 12; i++)
    {
        _jointLeg[i]->setTorque(_lowCmd->motorLegCmd[i].tau);
    }

    for (int i = 0; i < 6; i++)
    {
        _jointArm[i]->setTorque(_lowCmd->motorArmCmd[i].tau);
    }
}

bool IOWebots::isRunning()
{
    if (_supervisor->step(_timeStep) != -1)
        return true;
    else
        return false;
}

void IOWebots::initRecv()
{
    // supervisor init
    _robotNode = _supervisor->getFromDef(_supervisorName);
    if (_robotNode == NULL)
    {
        printf("error supervisor");
        exit(1);
    }

    // joystick init
    _joystick = _supervisor->getJoystick();
    _joystick->enable(_timeStep);

    // sensor init
    _imu = _supervisor->getInertialUnit(_imuName);
    _imu->enable(_timeStep);
    _gyro = _supervisor->getGyro(_gyroName);
    _gyro->enable(_timeStep);
    _accelerometer = _supervisor->getAccelerometer(_accelerometerName);
    _accelerometer->enable(_timeStep);

    for (int i = 0; i < 12; i++)
    {
        _jointSensorLeg[i] = _supervisor->getPositionSensor(_jointSensorLegName[i]);
        _jointSensorLeg[i]->enable(_timeStep);
    }

    for (int i = 0; i < 6; i++)
    {
        _jointSensorArm[i] = _supervisor->getPositionSensor(_jointSensorArmName[i]);
        _jointSensorArm[i]->enable(_timeStep);
    }

    // time step
    _lowState->timeStep = double(_timeStep) / 1000;
}

void IOWebots::initSend()
{
    for (int i = 0; i < 12; i++)
        _jointLeg[i] = _supervisor->getMotor(_jointLegName[i]);

    for (int i = 0; i < 6; i++)
        _jointArm[i] = _supervisor->getMotor(_jointArmName[i]);
}
