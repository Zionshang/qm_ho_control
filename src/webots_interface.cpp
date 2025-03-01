#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
{
    supervisor_ = new webots::Supervisor();
    time_step_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << time_step_ << std::endl;
    initRecv();
    initSend();

    last_leg_joint_position_ << 0.0, 0.0, 0.0, 0.0,
        0.67, 0.67, 0.67, 0.67,
        -1.3, -1.3, -1.3, -1.3;
    last_arm_joint_position_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

WebotsInterface::~WebotsInterface()
{
    delete supervisor_;
}

void WebotsInterface::recvState(LowState& low_state)
{
    // sensor
    const double *imuData = imu_->getQuaternion(); // x,y,z,w
    const double *gyroData = gyro_->getValues();
    const double *accelerometerData = accelerometer_->getValues();

    for (int i = 0; i < 3; i++)
    {
        low_state.imu.quaternion[i] = static_cast<double>(imuData[i]);
        low_state.imu.gyro[i] = static_cast<double>(gyroData[i]);
        low_state.imu.accelerometer[i] = static_cast<double>(accelerometerData[i]);
    }
    low_state.imu.quaternion[3] = static_cast<double>(imuData[3]);

    for (int i = 0; i < 12; i++)
    {
        low_state.motor_state_leg[i].q = joint_sensor_leg_[i]->getValue();
        low_state.motor_state_leg[i].dq = (low_state.motor_state_leg[i].q - last_leg_joint_position_(i)) / double(time_step_) * 1000;
        last_leg_joint_position_(i) = low_state.motor_state_leg[i].q;
    }

    for (int i = 0; i < 6; i++)
    {
        low_state.motor_state_arm[i].q = joint_sensor_arm_[i]->getValue();
        low_state.motor_state_arm[i].dq = (low_state.motor_state_arm[i].q - last_arm_joint_position_(i)) / double(time_step_) * 1000;
        last_arm_joint_position_(i) = low_state.motor_state_arm[i].q;
    }
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
        case 'a':
        case 'A':
            user_cmd.vel_body_B(1) += 0.5;
            break;
        case 'd':
        case 'D':
            user_cmd.vel_body_B(1) += -0.5;
            break;
        case 'j':
        case 'J':
            user_cmd.angvel_body_B(2) += 0.5;
            break;
        case 'l':
        case 'L':
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
        }
    }
    last_key_ = key_;
}

void WebotsInterface::sendCmd(LowCmd& low_cmd)
{
    for (int i = 0; i < 12; i++)
    {
        motor_leg_[i]->setTorque(low_cmd.motor_cmd_leg[i].tau);
    }

    for (int i = 0; i < 6; i++)
    {
        motor_arm_[i]->setTorque(low_cmd.motor_cmd_arm[i].tau);
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

    for (int i = 0; i < 12; i++)
    {
        joint_sensor_leg_[i] = supervisor_->getPositionSensor(joint_sensor_leg_name_[i]);
        joint_sensor_leg_[i]->enable(time_step_);
    }

    for (int i = 0; i < 6; i++)
    {
        joint_sensor_arm_[i] = supervisor_->getPositionSensor(joint_sensor_arm_name_[i]);
        joint_sensor_arm_[i]->enable(time_step_);
    }
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < 12; i++)
        motor_leg_[i] = supervisor_->getMotor(motor_leg_name_[i]);

    for (int i = 0; i < 6; i++)
        motor_arm_[i] = supervisor_->getMotor(motor_arm_name_[i]);
}
