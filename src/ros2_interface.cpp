#include "ros2_interface.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using std::placeholders::_1;

int Ros2Interface::kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

Ros2Interface::Ros2Interface(rclcpp::Node::SharedPtr node) : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "MujocoInterface已初始化.");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos, std::bind(&Ros2Interface::joint_state_callback, this, _1));
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", qos, std::bind(&Ros2Interface::imu_callback, this, _1));
    actuator_cmd_publisher_ = node_->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
}

void Ros2Interface::recvState(LowState &low_state)
{
    low_state = low_state_;
}

void Ros2Interface::recvUserCmd(UserCommand &user_cmd)
{
    if (kbhit())
    {
        key_ = getchar();
    }

    if (key_ != last_key_)
    {
        switch (key_)
        {
        case 'w':
        case 'W':
            user_cmd.vel_body_B(0) += 0.5;
            break;
        case 'b':
        case 'B':
            user_cmd.vel_body_B(0) += -0.5;
            break;
        case 'q':
        case 'Q':
            user_cmd.vel_body_B(1) += 0.5;
            break;
        case 'e':
        case 'E':
            user_cmd.vel_body_B(1) += -0.5;
            break;
        case 'a':
        case 'A':
            user_cmd.angvel_body_B(2) += 0.5;
            break;
        case 'd':
        case 'D':
            user_cmd.angvel_body_B(2) += -0.5;
            break;
        case 's':
        case 'S':
            user_cmd.vel_body_B.setZero();
            user_cmd.angvel_body_B.setZero();
            break;
        case '1':
            user_cmd.gait_name = GaitName::STANCE;
            break;
        case '2':
            user_cmd.gait_name = GaitName::TROT;
            break;
        case '3':
            user_cmd.gait_name = GaitName::WALKING_TROT;
            break;
        case '4':
            user_cmd.gait_name = GaitName::RUNNING_TROT;
            break;
        case '5':
            user_cmd.gait_name = GaitName::WALK;
            break;
        case 'r':
        case 'R':
            user_cmd.arm_joint_pos(0) += 0.05;
            break;
        case 'f':
        case 'F':
            user_cmd.arm_joint_pos(0) -= 0.05;
            break;
        case 't':
        case 'T':
            user_cmd.arm_joint_pos(1) += 0.05;
            break;
        case 'g':
        case 'G':
            user_cmd.arm_joint_pos(1) -= 0.05;
            break;
        case 'y':
        case 'Y':
            user_cmd.arm_joint_pos(2) += 0.05;
            break;
        case 'h':
        case 'H':
            user_cmd.arm_joint_pos(2) -= 0.05;
            break;
        case 'u':
        case 'U':
            user_cmd.arm_joint_pos(3) += 0.05;
            break;
        case 'j':
        case 'J':
            user_cmd.arm_joint_pos(3) -= 0.05;
            break;
        case 'i':
        case 'I':
            user_cmd.arm_joint_pos(4) += 0.05;
            break;
        case 'k':
        case 'K':
            user_cmd.arm_joint_pos(4) -= 0.05;
            break;
        case 'o':
        case 'O':
            user_cmd.arm_joint_pos(5) += 0.05;
            break;
        case 'l':
        case 'L':
            user_cmd.arm_joint_pos(5) -= 0.05;
            break;
        case '9':
            user_cmd.ctrl_type = ControllerType::POSITION_STAND_UP;
            break;
        case '8':
            user_cmd.ctrl_type = ControllerType::TORQUE_CONTROLLER;
            break;
        case '7':
            user_cmd.ctrl_type = ControllerType::POSITION_LIE_DOWN;
            break;
        }
    }
    last_key_ = key_;
}
void Ros2Interface::sendCmd(LowCmd &low_cmd)
{
    std::cout << "----------" << std::endl;
    auto message = custom_msgs::msg::ActuatorCmds();
    for (int i = 0; i < 12; i++)
    {
        message.actuators_name.push_back(actuator_names_[i]);
        message.kp.push_back(low_cmd.motor_cmd_leg[i].kp);
        message.pos.push_back(low_cmd.motor_cmd_leg[i].q);
        message.kd.push_back(low_cmd.motor_cmd_leg[i].kd);
        message.vel.push_back(low_cmd.motor_cmd_leg[i].dq);
        message.torque.push_back(low_cmd.motor_cmd_leg[i].tau);
    }
    for (int i = 0; i < 6; i++)
    {
        message.actuators_name.push_back(actuator_names_[12 + i]);
        message.kp.push_back(low_cmd.motor_cmd_arm[i].kp);
        message.pos.push_back(low_cmd.motor_cmd_arm[i].q);
        message.kd.push_back(low_cmd.motor_cmd_arm[i].kd);
        message.vel.push_back(low_cmd.motor_cmd_arm[i].dq);
        message.torque.push_back(low_cmd.motor_cmd_arm[i].tau);
        std::cout << actuator_names_[12 + i] << ": "
                  << low_cmd.motor_cmd_arm[i].q
                  << low_cmd.motor_cmd_arm[i].dq
                  << low_cmd.motor_cmd_arm[i].tau << std::endl;
    }
    std::cout << std::endl;
    actuator_cmd_publisher_->publish(message);
}

bool Ros2Interface::isRunning()
{
    return true;
}

void Ros2Interface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < 12; i++)
    {
        low_state_.motor_state_leg[i].q = msg->position[i];
        low_state_.motor_state_leg[i].dq = msg->velocity[i];
    }
    std::cout << std::endl;

    for (int i = 0; i < 6; i++)
    {
        low_state_.motor_state_arm[i].q = msg->position[12 + i];
        low_state_.motor_state_arm[i].dq = msg->velocity[12 + i];
    }
}

void Ros2Interface::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 存储四元数
    low_state_.imu.quaternion[0] = msg->orientation.x;
    low_state_.imu.quaternion[1] = msg->orientation.y;
    low_state_.imu.quaternion[2] = msg->orientation.z;
    low_state_.imu.quaternion[3] = msg->orientation.w;

    // 存储角速度
    low_state_.imu.gyro[0] = msg->angular_velocity.x;
    low_state_.imu.gyro[1] = msg->angular_velocity.y;
    low_state_.imu.gyro[2] = msg->angular_velocity.z;

    // 存储加速度
    low_state_.imu.accelerometer[0] = msg->linear_acceleration.x;
    low_state_.imu.accelerometer[1] = msg->linear_acceleration.y;
    low_state_.imu.accelerometer[2] = msg->linear_acceleration.z;
}
