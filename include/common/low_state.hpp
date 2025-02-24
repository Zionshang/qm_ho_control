#pragma once

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorState
{
    double q;
    double dq;

    MotorState()
    {
        q = 0;
        dq = 0;
    }
};

struct Supervisor
{
    double robot_pos[3]; // position of robot node in webots
    double robot_vel[3]; // velocity of robot node in webots

    Supervisor()
    {
        for (int i = 0; i < 3; i++)
        {
            robot_pos[i] = 0;
            robot_vel[i] = 0;
        }
    }
};

struct IMU
{
    double quaternion[4]; // x,y,z,w
    double gyro[3];
    double accelerometer[3];

    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyro[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 1;
    }

    Quaterniond getQuaternion()
    {
        return Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    }
    Vector3d getGyro()
    {
        return Vector3d(gyro[0], gyro[1], gyro[2]);
    }
    Vector3d getAccelerometer()
    {
        return Vector3d(accelerometer[0], accelerometer[1], accelerometer[2]);
    }
};

struct LowState
{
    IMU imu;
    Supervisor supervisor;
    MotorState motor_state_leg[12];
    MotorState motor_state_arm[6];

    // todo: 是否需要换成12列
    Matrix34d getLegJointPosition()
    {
        Matrix34d q_leg;
        for (int i = 0; i < 4; i++)
        {
            q_leg.col(i)(0) = motor_state_leg[3 * i].q;
            q_leg.col(i)(1) = motor_state_leg[3 * i + 1].q;
            q_leg.col(i)(2) = motor_state_leg[3 * i + 2].q;
        }
        return q_leg;
    }

    Matrix34d getLegJointVelocity()
    {
        Matrix34d dq_leg;
        for (int i = 0; i < 4; i++)
        {
            dq_leg.col(i)(0) = motor_state_leg[3 * i].dq;
            dq_leg.col(i)(1) = motor_state_leg[3 * i + 1].dq;
            dq_leg.col(i)(2) = motor_state_leg[3 * i + 2].dq;
        }
        return dq_leg;
    }

    Vector6d getArmJointPosition()
    {
        Vector6d q_arm;
        for (int i = 0; i < 6; i++)
            q_arm(i) = motor_state_arm[i].q;
        return q_arm;
    }

    Vector6d getArmJointVelocity()
    {
        Vector6d dq_arm;
        for (int i = 0; i < 6; i++)
            dq_arm(i) = motor_state_arm[i].dq;
        return dq_arm;
    }

    Quaterniond getQuaternion() { return imu.getQuaternion(); }
    Vector3d getGyro() { return imu.getGyro(); }
    Vector3d getAccelerometer() { return imu.getAccelerometer(); }
};
