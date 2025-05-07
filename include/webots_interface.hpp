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

    std::string supervisor_name_ = "bqr3_arm";
    std::string imu_name_ = "imu_link_imu";
    std::string gyro_name_ = "imu_link_gyro";
    std::string accelerometer_name_ = "imu_link_accelerometer";
    std::vector<std::string> motor_sensor_name_ = {"FL_abd_joint_sensor", "FL_hip_joint_sensor", "FL_knee_joint_sensor",
                                                   "FR_abd_joint_sensor", "FR_hip_joint_sensor", "FR_knee_joint_sensor",
                                                   "HL_abd_joint_sensor", "HL_hip_joint_sensor", "HL_knee_joint_sensor",
                                                   "HR_abd_joint_sensor", "HR_hip_joint_sensor", "HR_knee_joint_sensor",
                                                   "shoulder_joint_sensor", "upper_arm_joint_sensor", "forearm_joint_sensor", "wrist_pitch_joint_sensor", "wrist_yall_joint_sensor"};
    std::vector<std::string> motor_name_ = {"FL_abd_joint", "FL_hip_joint", "FL_knee_joint",
                                            "FR_abd_joint", "FR_hip_joint", "FR_knee_joint",
                                            "HL_abd_joint", "HL_hip_joint", "HL_knee_joint",
                                            "HR_abd_joint", "HR_hip_joint", "HR_knee_joint",
                                            "shoulder_joint", "upper_arm_joint", "forearm_joint", "wrist_pitch_joint", "wrist_yall_joint"};

    webots::Node *robot_node_; // used for cheat robot position

    int key_, last_key_;
    VectorXd last_motor_position_;
};
