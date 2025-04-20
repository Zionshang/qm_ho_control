#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
{
    supervisor_ = new webots::Supervisor();
    time_step_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << time_step_ << std::endl;
    initRecv();
    initSend();

    // TODO: 使用参数传递
    last_leg_joint_position_ << 0.0, 0.0, 0.0, 0.0,
        0.72, 0.72, 0.72, 0.72,
        -1.44, -1.44, -1.44, -1.44;
    last_arm_joint_position_ << 0, -1.57, 2.88, 0.26, 0;

    robot_node_ = supervisor_->getFromDef(supervisor_name_);
    if (robot_node_ == NULL)
    {
        printf("error supervisor");
        exit(1);
    }
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

    for (int i = 0; i < NUM_LEG_MOTOR; i++)
    {
        low_state_.motor_state_leg[i].q = joint_sensor_leg_[i]->getValue();
        low_state_.motor_state_leg[i].dq = (low_state_.motor_state_leg[i].q - last_leg_joint_position_(i)) / double(time_step_) * 1000;
        last_leg_joint_position_(i) = low_state_.motor_state_leg[i].q;
    }

    for (int i = 0; i < NUM_ARM_MOTOR; i++)
    {
        low_state_.motor_state_arm[i].q = joint_sensor_arm_[i]->getValue();
        low_state_.motor_state_arm[i].dq = (low_state_.motor_state_arm[i].q - last_arm_joint_position_(i)) / double(time_step_) * 1000;
        last_arm_joint_position_(i) = low_state_.motor_state_arm[i].q;
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
    for (int i = 0; i < NUM_LEG_MOTOR; i++)
    {
        tau = low_cmd.motor_cmd_leg[i].tau +
              low_cmd.motor_cmd_leg[i].kp * (low_cmd.motor_cmd_leg[i].q - low_state_.motor_state_leg[i].q) +
              low_cmd.motor_cmd_leg[i].kd * (low_cmd.motor_cmd_leg[i].dq - low_state_.motor_state_leg[i].dq);
        motor_leg_[i]->setTorque(tau);
    }
    for (int i = 0; i < NUM_ARM_MOTOR; i++)
    {
        tau = low_cmd.motor_cmd_arm[i].tau +
              low_cmd.motor_cmd_arm[i].kp * (low_cmd.motor_cmd_arm[i].q - low_state_.motor_state_arm[i].q) +
              low_cmd.motor_cmd_arm[i].kd * (low_cmd.motor_cmd_arm[i].dq - low_state_.motor_state_arm[i].dq);
        // std::cout << low_cmd.motor_cmd_arm[i].q << "  "
        //           << low_state_.motor_state_arm[i].q << "  "
        //           << low_cmd.motor_cmd_arm[i].dq << "  "
        //           << low_state_.motor_state_arm[i].dq << "  "
        //           << std::endl;
        motor_arm_[i]->setTorque(tau);
    }
}

bool WebotsInterface::isRunning()
{
    if (supervisor_->step(time_step_) != -1)
        return true;
    else
        return false;
}

void WebotsInterface::initRecv()
{
    // // joystick init
    // _joystick = supervisor_->getJoystick();
    // _joystick->enable(time_step_);

    // keyboard init
    keyboard_ = supervisor_->getKeyboard();
    keyboard_->enable(time_step_);

    // sensor init
    imu_ = supervisor_->getInertialUnit(imu_name_);
    imu_->enable(time_step_);
    gyro_ = supervisor_->getGyro(gyro_name_);
    gyro_->enable(time_step_);
    accelerometer_ = supervisor_->getAccelerometer(accelerometer_name_);
    accelerometer_->enable(time_step_);

    for (int i = 0; i < NUM_LEG_MOTOR; i++)
    {
        joint_sensor_leg_[i] = supervisor_->getPositionSensor(joint_sensor_leg_name_[i]);
        joint_sensor_leg_[i]->enable(time_step_);
    }

    for (int i = 0; i < NUM_ARM_MOTOR; i++)
    {
        joint_sensor_arm_[i] = supervisor_->getPositionSensor(joint_sensor_arm_name_[i]);
        joint_sensor_arm_[i]->enable(time_step_);
    }
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < NUM_LEG_MOTOR; i++)
        motor_leg_[i] = supervisor_->getMotor(motor_leg_name_[i]);

    for (int i = 0; i < NUM_ARM_MOTOR; i++)
        motor_arm_[i] = supervisor_->getMotor(motor_arm_name_[i]);
}
