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

    LowState low_state_;
    
    // webots interface
    webots::Supervisor *supervisor_;
    webots::Motor *motor_leg_[12];
    webots::PositionSensor *joint_sensor_leg_[12];
    webots::InertialUnit *imu_;
    webots::Gyro *gyro_;
    // webots::Joystick *_joystick;
    webots::Accelerometer *accelerometer_;
    webots::Keyboard *keyboard_;

    std::string supervisor_name_ = "galileo_mini_x5";
    std::string imu_name_ = "imu_link_imu";
    std::string gyro_name_ = "imu_link_gyro";
    std::string accelerometer_name_ = "imu_link_accelerometer";
    std::string joint_sensor_leg_name_[12] = {"FL_abd_joint_sensor", "FL_hip_joint_sensor", "FL_knee_joint_sensor",
                                              "FR_abd_joint_sensor", "FR_hip_joint_sensor", "FR_knee_joint_sensor",
                                              "HL_abd_joint_sensor", "HL_hip_joint_sensor", "HL_knee_joint_sensor",
                                              "HR_abd_joint_sensor", "HR_hip_joint_sensor", "HR_knee_joint_sensor"};
    std::string motor_leg_name_[12] = {"FL_abd_joint", "FL_hip_joint", "FL_knee_joint",
                                       "FR_abd_joint", "FR_hip_joint", "FR_knee_joint",
                                       "HL_abd_joint", "HL_hip_joint", "HL_knee_joint",
                                       "HR_abd_joint", "HR_hip_joint", "HR_knee_joint"};

    webots::Motor *motor_arm_[6];
    webots::PositionSensor *joint_sensor_arm_[6];
    std::string joint_sensor_arm_name_[6] = {"joint1_sensor", "joint2_sensor", "joint3_sensor",
                                             "joint4_sensor", "joint5_sensor", "joint6_sensor"};
    std::string motor_arm_name_[6] = {"joint1", "joint2", "joint3",
                                      "joint4", "joint5", "joint6"};

    int key_, last_key_;
};
