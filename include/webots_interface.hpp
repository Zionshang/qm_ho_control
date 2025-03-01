#pragma once

#include "common/low_cmd.hpp"
#include "common/low_state.hpp"
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

class WebotsInterface
{
public:
    WebotsInterface();
    ~WebotsInterface();
    void recvState(LowState &low_state);
    void recvUserCmd(UserCommand &user_cmd);
    void sendCmd(LowCmd &low_cmd);
    bool isRunning();

    // get current time in seconds
    double current_time() { return supervisor_->getTime(); }

private:
    void initRecv();
    void initSend();

    int time_step_;
    Matrix34d last_leg_joint_position_;
    Vector6d last_arm_joint_position_;

    // webots interface
    webots::Supervisor *supervisor_;
    webots::Motor *motor_leg_[12];
    webots::PositionSensor *joint_sensor_leg_[12];
    webots::InertialUnit *imu_;
    webots::Gyro *gyro_;
    // webots::Joystick *_joystick;
    webots::Accelerometer *accelerometer_;
    webots::Keyboard *keyboard_;

    std::string supervisor_name_ = "AliengoZ1";
    std::string imu_name_ = "trunk_imu inertial";
    std::string gyro_name_ = "trunk_imu gyro";
    std::string accelerometer_name_ = "trunk_imu accelerometer";
    std::string joint_sensor_leg_name_[12] = {"leg1_hip_joint_sensor", "leg1_thigh_joint_sensor", "leg1_calf_joint_sensor",
                                              "leg2_hip_joint_sensor", "leg2_thigh_joint_sensor", "leg2_calf_joint_sensor",
                                              "leg3_hip_joint_sensor", "leg3_thigh_joint_sensor", "leg3_calf_joint_sensor",
                                              "leg4_hip_joint_sensor", "leg4_thigh_joint_sensor", "leg4_calf_joint_sensor"};
    std::string motor_leg_name_[12] = {"leg1_hip_joint", "leg1_thigh_joint", "leg1_calf_joint",
                                      "leg2_hip_joint", "leg2_thigh_joint", "leg2_calf_joint",
                                      "leg3_hip_joint", "leg3_thigh_joint", "leg3_calf_joint",
                                      "leg4_hip_joint", "leg4_thigh_joint", "leg4_calf_joint"};

    webots::Motor *motor_arm_[6];
    webots::PositionSensor *joint_sensor_arm_[6];
    std::string joint_sensor_arm_name_[6] = {"joint1_sensor", "joint2_sensor", "joint3_sensor",
                                             "joint4_sensor", "joint5_sensor", "joint6_sensor"};
    std::string motor_arm_name_[6] = {"joint1", "joint2", "joint3",
                                      "joint4", "joint5", "joint6"};

    int key_, last_key_;
};
