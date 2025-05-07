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
    void motorTest();
    bool isRunning();

    // get current time in seconds
    double current_time() { return supervisor_->getTime(); }
    const webots::Node *CheatNode() { return robot_node_; }

private:
    void initRecv();
    void initSend();

    int num_motor_;
    int timestep_;

    LowState low_state_;

    // webots interface
    webots::Supervisor *supervisor_;
    std::vector<webots::Motor *> motor_;
    std::vector<webots::PositionSensor *> motor_sensor_;
    webots::InertialUnit *imu_;
    webots::Gyro *gyro_;
    // webots::Joystick *_joystick;
    webots::Accelerometer *accelerometer_;
    webots::Keyboard *keyboard_;

    std::string supervisor_name_ = "GalileoV1d6X5";
    std::string imu_name_ = "imu";
    std::string gyro_name_ = "gyro";
    std::string accelerometer_name_ = "accelerometer";
    std::vector<std::string> motor_name_ = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::vector<std::string> motor_sensor_name_ = {"FL_hip_joint_sensor", "FL_thigh_joint_sensor", "FL_calf_joint_sensor",
                                                   "FR_hip_joint_sensor", "FR_thigh_joint_sensor", "FR_calf_joint_sensor",
                                                   "RL_hip_joint_sensor", "RL_thigh_joint_sensor", "RL_calf_joint_sensor",
                                                   "RR_hip_joint_sensor", "RR_thigh_joint_sensor", "RR_calf_joint_sensor",
                                                   "joint1_sensor", "joint2_sensor", "joint3_sensor", "joint4_sensor", "joint5_sensor", "joint6_sensor"};
    webots::Node *robot_node_; // used for cheat robot position

    int key_, last_key_;
    VectorXd last_motor_position_;
};
