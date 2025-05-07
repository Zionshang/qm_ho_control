#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
{
    supervisor_ = new webots::Supervisor();
    timestep_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << timestep_ << std::endl;
    num_motor_ = motor_name_.size();

    motor_sensor_.assign(num_motor_, NULL);
    motor_.assign(num_motor_, NULL);

    initRecv();
    initSend();

    // TODO: 使用参数传递
    last_motor_position_.setZero(num_motor_);
    last_motor_position_ << 0.0, 0.72, -1.44,
        0.0, 0.72, -1.44,
        0.0, 0.72, -1.44,
        0.0, 0.72, -1.44,
        0, -1.57, 2.88, 0.26, 0;

    robot_node_ = supervisor_->getFromDef(supervisor_name_);
    if (robot_node_ == NULL)
    {
        std::cerr << "ERROR: Failed to get robot node with supervisor name '" << supervisor_name_ << "'" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::cout << "test" << std::endl;
}

WebotsInterface::~WebotsInterface()
{
    delete supervisor_;
}

void WebotsInterface::recvState(LowState &low_state)
{
    // sensor
    const double *imuData = imu_->getQuaternion(); // x,y,z,w
    const double *gyroData = gyro_->getValues();
    const double *accelerometerData = accelerometer_->getValues();

    for (int i = 0; i < 3; i++)
    {
        low_state_.imu.quaternion[i] = static_cast<double>(imuData[i]);
        low_state_.imu.gyro[i] = static_cast<double>(gyroData[i]);
        low_state_.imu.accelerometer[i] = static_cast<double>(accelerometerData[i]);
    }
    low_state_.imu.quaternion[3] = static_cast<double>(imuData[3]);

    for (int i = 0; i < num_motor_; i++)
    {
        if (i < 12)
        {
            low_state_.motor_state_leg[i].q = motor_sensor_[i]->getValue();
            low_state_.motor_state_leg[i].dq = (low_state_.motor_state_leg[i].q - last_motor_position_(i)) / double(timestep_) * 1000;
            last_motor_position_(i) = low_state_.motor_state_leg[i].q;
        }
        else
        {
            low_state_.motor_state_arm[i - 12].q = motor_sensor_[i]->getValue();
            low_state_.motor_state_arm[i - 12].dq = (low_state_.motor_state_arm[i - 12].q - last_motor_position_(i)) / double(timestep_) * 1000;
            last_motor_position_(i) = low_state_.motor_state_arm[i - 12].q;
        }
    }
    low_state = low_state_;
}

void WebotsInterface::recvUserCmd(UserCommand &user_cmd)
{
    key_ = keyboard_->getKey();
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

        case 315: // arrow upper
            user_cmd.angvel_body_B(1) += 0.1;
            break;
        case 317: // arrow upper
            user_cmd.angvel_body_B(1) -= 0.1;
            break;
        }
    }
    last_key_ = key_;
}

void WebotsInterface::sendCmd(LowCmd &low_cmd)
{
    double tau;
    for (int i = 0; i < num_motor_; i++)
    {
        if (i < 12)
        {

            tau = low_cmd.motor_cmd_leg[i].tau +
                  low_cmd.motor_cmd_leg[i].kp * (low_cmd.motor_cmd_leg[i].q - low_state_.motor_state_leg[i].q) +
                  low_cmd.motor_cmd_leg[i].kd * (low_cmd.motor_cmd_leg[i].dq - low_state_.motor_state_leg[i].dq);
            motor_[i]->setTorque(tau);
        }
        else
        {
            tau = low_cmd.motor_cmd_arm[i - 12].tau +
                  low_cmd.motor_cmd_arm[i - 12].kp * (low_cmd.motor_cmd_arm[i - 12].q - low_state_.motor_state_arm[i - 12].q) +
                  low_cmd.motor_cmd_arm[i - 12].kd * (low_cmd.motor_cmd_arm[i - 12].dq - low_state_.motor_state_arm[i - 12].dq);
            motor_[i]->setTorque(tau);
        }
    }
}

bool WebotsInterface::isRunning()
{
    if (supervisor_->step(timestep_) != -1)
        return true;
    else
        return false;
}

void WebotsInterface::initRecv()
{
    // // joystick init
    // _joystick = supervisor_->getJoystick();
    // _joystick->enable(timestep_);

    // keyboard init
    keyboard_ = supervisor_->getKeyboard();
    keyboard_->enable(timestep_);

    // sensor init
    imu_ = supervisor_->getInertialUnit(imu_name_);
    imu_->enable(timestep_);
    gyro_ = supervisor_->getGyro(gyro_name_);
    gyro_->enable(timestep_);
    accelerometer_ = supervisor_->getAccelerometer(accelerometer_name_);
    accelerometer_->enable(timestep_);

    for (int i = 0; i < num_motor_; i++)
    {
        motor_sensor_[i] = supervisor_->getPositionSensor(motor_sensor_name_[i]);
        if (motor_sensor_[i] == NULL)
        {
            std::cerr << "ERROR: Failed to get motor sensor with name '" << motor_sensor_name_[i] << "'" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        motor_sensor_[i]->enable(timestep_);
    }
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < num_motor_; i++)
    {
        motor_[i] = supervisor_->getMotor(motor_name_[i]);
        if (motor_[i] == NULL)
        {
            std::cerr << "ERROR: Failed to get motor with name '" << motor_name_[i] << "'" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
}