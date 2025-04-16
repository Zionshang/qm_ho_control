#pragma once

#include "common/low_cmd.hpp"
#include "common/low_state.hpp"
#include "ctrl_component.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_msgs/msg/actuator_cmds.hpp>
#include <sensor_msgs/msg/imu.hpp>

class Ros2Interface
{
public:
    Ros2Interface(rclcpp::Node::SharedPtr node);

    void recvState(LowState &low_state);
    void recvUserCmd(UserCommand &user_cmd);
    void sendCmd(LowCmd &low_cmd);
    bool isRunning();

private:

    rclcpp::Node::SharedPtr node_; // 存储节点指针

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_publisher_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    int kbhit();

    int key_, last_key_;
    LowState low_state_;
    std::vector<std::string> actuator_names_ = {"FL_abd_joint", "FL_hip_joint", "FL_knee_joint",
                                                "FR_abd_joint", "FR_hip_joint", "FR_knee_joint",
                                                "HL_abd_joint", "HL_hip_joint", "HL_knee_joint",
                                                "HR_abd_joint", "HR_hip_joint", "HR_knee_joint",
                                                "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
};
