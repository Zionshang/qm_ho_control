#pragma once

#include "common/LowCmd.h"
#include "common/LowState.h"
#include "ctrl_component.hpp"
#include <iostream>
#include <string>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
// #include <webots/Joystick.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>

class IOWebots
{
public:
    IOWebots(LowState *lowState, LowCmd *lowCmd);
    ~IOWebots();
    void recvState();
    void recvUserCmd(UserCommand &user_cmd);
    void sendCmd();
    bool isRunning();

private:
    void initRecv();
    void initSend();

    int _timeStep;
    Matrix34d _lastqLeg;

    LowState *_lowState;
    LowCmd *_lowCmd;

    // webots interface
    webots::Supervisor *_supervisor;
    webots::Node *_robotNode;
    webots::Motor *_jointLeg[12];
    webots::PositionSensor *_jointSensorLeg[12];
    webots::InertialUnit *_imu;
    webots::Gyro *_gyro;
    // webots::Joystick *_joystick;
    webots::Accelerometer *_accelerometer;
    webots::Keyboard *keyboard_;

    std::string _supervisorName = "AliengoZ1";
    std::string _imuName = "trunk_imu inertial";
    std::string _gyroName = "trunk_imu gyro";
    std::string _accelerometerName = "trunk_imu accelerometer";
    std::string _jointSensorLegName[12] = {"leg1_hip_joint_sensor", "leg1_thigh_joint_sensor", "leg1_calf_joint_sensor",
                                           "leg2_hip_joint_sensor", "leg2_thigh_joint_sensor", "leg2_calf_joint_sensor",
                                           "leg3_hip_joint_sensor", "leg3_thigh_joint_sensor", "leg3_calf_joint_sensor",
                                           "leg4_hip_joint_sensor", "leg4_thigh_joint_sensor", "leg4_calf_joint_sensor"};
    std::string _jointLegName[12] = {"leg1_hip_joint", "leg1_thigh_joint", "leg1_calf_joint",
                                     "leg2_hip_joint", "leg2_thigh_joint", "leg2_calf_joint",
                                     "leg3_hip_joint", "leg3_thigh_joint", "leg3_calf_joint",
                                     "leg4_hip_joint", "leg4_thigh_joint", "leg4_calf_joint"};

    Vector6d _lastqArm;

    webots::Motor *_jointArm[6];
    webots::PositionSensor *_jointSensorArm[6];

    std::string _jointSensorArmName[6] = {"joint1_sensor", "joint2_sensor", "joint3_sensor",
                                          "joint4_sensor", "joint5_sensor", "joint6_sensor"};
    std::string _jointArmName[6] = {"joint1", "joint2", "joint3",
                                    "joint4", "joint5", "joint6"};

    int key_, last_key;
};
