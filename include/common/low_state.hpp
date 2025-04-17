#pragma once

#include "common/types.hpp"
#include "utils/math_tools.hpp"

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
};

struct LowState
{
    IMU imu;
    MotorState motor_state_leg[12];
    MotorState motor_state_arm[5];

    // todo: 是否需要换成12列
    const Matrix34d getLegJointPosition() const
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

    const Matrix34d getLegJointVelocity() const
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

    const VectorXd getArmJointPosition() const
    {
        VectorXd q_arm(5); // todo: 不使用魔法数字
        for (int i = 0; i < 5; i++)
            q_arm(i) = motor_state_arm[i].q;
        return q_arm;
    }

    const VectorXd getArmJointVelocity() const
    {
        VectorXd dq_arm(5); // todo: 不使用魔法数字
        for (int i = 0; i < 5; i++)
            dq_arm(i) = motor_state_arm[i].dq;
        return dq_arm;
    }

    const Quaterniond getQuaternion() const
    {
        const auto &quaternion = imu.quaternion;
        return Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    }
    const Vector3d getGyro() const
    {
        const auto &gyro = imu.gyro;
        return Vector3d(gyro[0], gyro[1], gyro[2]);
    }
    const Vector3d getAccelerometer() const
    {
        const auto &accelerometer = imu.accelerometer;
        return Vector3d(accelerometer[0], accelerometer[1], accelerometer[2]);
    }
};
