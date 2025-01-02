#pragma once

#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "common/enumClass.h"

struct MotorState
{
    float q;
    float dq;

    MotorState()
    {
        q = 0;
        dq = 0;
    }
};

struct Supervisor
{
    float robotPos[3]; // position of robot node in webots
    float robotVel[3]; // velocity of robot node in webots

    Supervisor()
    {
        for (int i = 0; i < 3; i++)
        {
            robotPos[i] = 0;
            robotVel[i] = 0;
        }
    }
};

struct IMU
{
    float quaternion[4]; // x,y,z,w
    float gyro[3];

    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyro[i] = 0;
        }
        quaternion[3] = 1;
    }

    Quat getQuaternion()
    {
        return Quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    }
    Vec3 getGyro()
    {
        return Vec3(gyro[0], gyro[1], gyro[2]);
    }
};

struct UserValue
{
    float lx;
    float ly;
    float rx;
    float ry;
    float z;
    UserValue() { setZero(); }
    void setZero()
    {
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        z = 0;
    }
};

struct LowState
{
    IMU imu;
    Supervisor supervisor;
    MotorState motorLeg[12];
    UserValue userValue;
    UserCommand userCmd;
    double timeStep;
    double currentTime;
    MotorState motorArm[6];

    LowState()
    {
        userCmd = UserCommand::B;
        timeStep = 0.0;
        currentTime = 0.0;
    }

    Vec34 getQLeg()
    {
        Vec34 qLegs;
        for (int i = 0; i < 4; i++)
        {
            qLegs.col(i)(0) = motorLeg[3 * i].q;
            qLegs.col(i)(1) = motorLeg[3 * i + 1].q;
            qLegs.col(i)(2) = motorLeg[3 * i + 2].q;
        }
        return qLegs;
    }

    Vec34 getDqLeg()
    {
        Vec34 qdLegs;
        for (int i = 0; i < 4; i++)
        {
            qdLegs.col(i)(0) = motorLeg[3 * i].dq;
            qdLegs.col(i)(1) = motorLeg[3 * i + 1].dq;
            qdLegs.col(i)(2) = motorLeg[3 * i + 2].dq;
        }
        return qdLegs;
    }

    Vec6 getQArm()
    {
        Vec6 qArm;
        for (int i = 0; i < 6; i++)
            qArm(i) = motorArm[i].q;
        return qArm;
    }

    Vec6 getDqArm()
    {
        Vec6 dqArm;
        for (int i = 0; i < 6; i++)
            dqArm(i) = motorArm[i].dq;
        return dqArm;
    }

    Quat getQuaternion() { return imu.getQuaternion(); }
    Vec3 getGyro() { return imu.getGyro(); }
    double getCurrentTime() { return currentTime; }
    double getTimeStep() { return timeStep; }
    UserCommand getUserCmd() { return userCmd; }
};
