#include "kalman_filter_estimator.hpp"

KalmanFilterEstimator::KalmanFilterEstimator(shared_ptr<PinocchioInterface> pin_interface, double dt)
    : pin_interface_(pin_interface)
{
    // 初始化状态向量
    x_hat_.setZero(kStateDim);
    x_hat_(2) = 0.42;

    // 初始化状态矩阵
    A_.setIdentity();
    A_.block<3, 3>(0, 3) = dt * kIdentity3_;
    B_.block<3, 3>(0, 0) = 0.5 * dt * dt * kIdentity3_;
    B_.block<3, 3>(3, 0) = dt * kIdentity3_;

    // 初始化测量矩阵
    H_.setZero();
    for (size_t i = 0; i < kFeetNum; ++i)
    {
        H_.block<3, 3>(3 * i, 0) = kIdentity3_;
        H_.block<3, 3>(3 * (kFeetNum + i), 3) = kIdentity3_;
        H_(2 * kFeetDim + i, 6 + 3 * i + 2) = 1.0;
    }
    H_.block(0, 6, kFeetDim, kFeetDim) = -MatrixXd::Identity(kFeetDim, kFeetDim);

    // 初始化过程噪声协方差矩阵
    Q_init_.setIdentity();
    Q_init_.block<3, 3>(0, 0) = (dt / 20.) * kIdentity3_ * noise_processimu_position_;
    Q_init_.block<3, 3>(3, 3) = (dt * 9.81 / 20.) * kIdentity3_ * noise_processimu_velocity_;
    Q_init_.block<kFeetDim, kFeetDim>(6, 6) = dt * MatrixXd::Identity(kFeetDim, kFeetDim) * noise_process_foot_position_;
    Q_ = Q_init_;
    Q_(2, 2) = kLargeVariance_;

    // 初始化测量噪声协方差矩阵
    R_init_.setIdentity();
    R_init_.block<kFeetDim, kFeetDim>(0, 0) *= noise_meas_joint_position_;
    R_init_.block<kFeetDim, kFeetDim>(kFeetDim, kFeetDim) *= noise_meas_joint_velocity_;
    R_init_.block<kFeetNum, kFeetNum>(2 * kFeetDim, 2 * kFeetDim) *= noise_meas_foot_height_;
    R_ = R_init_;

    // 初始化预测误差
    P_ = 100. * Q_init_;
}
void KalmanFilterEstimator::update(const LowState &low_state, const Vector4i &contact_flag, RobotState &robot_state)
{
    // body
    auto &body_state = robot_state.body;
    body_state.pos = x_hat_.segment<3>(0);
    body_state.vel = x_hat_.segment<3>(3);
    body_state.quat = low_state.getQuaternion();
    RotMat rotmat_body_ = body_state.quat.toRotationMatrix();
    body_state.angvel = rotmat_body_ * low_state.getGyro();

    // joint
    auto &joint_state = robot_state.joint;
    joint_state.pos_leg = low_state.getLegJointPosition();
    joint_state.vel_leg = low_state.getLegJointVelocity();
    joint_state.pos_arm = low_state.getArmJointPosition();
    joint_state.vel_arm = low_state.getArmJointVelocity();

    // update covariance matix because of swing leg
    updateCovarianceMatrix(contact_flag);

    // update measurement
    robot_state.pos_gen << body_state.pos,
        body_state.quat.coeffs(),
        mat34ToVec12(joint_state.pos_leg), joint_state.pos_arm;
    robot_state.vel_gen << rotmat_body_.transpose() * body_state.vel,
        rotmat_body_.transpose() * body_state.angvel,
        mat34ToVec12(joint_state.vel_leg), joint_state.vel_arm;
    updateMeasurement(robot_state);

    // update state transition
    Vector3d acc_body = rotmat_body_ * low_state.getAccelerometer() + kGravity_;
    x_hat_ = A_ * x_hat_ + B_ * acc_body;

    // update priori prediction covariance
    P_priori_ = A_ * P_ * A_.transpose() + Q_;

    // update kalman gain
    H_T_ = H_.transpose();
    S_ = H_ * P_priori_ * H_T_ + R_;
    S_lu_ = S_.lu();
    S_z_ = S_lu_.solve(z_ - H_ * x_hat_);

    // uptate state
    x_hat_ += P_priori_ * H_T_ * S_z_;

    // update prediction covariance
    S_H_ = S_lu_.solve(H_);
    P_ = (MatrixXd::Identity(kStateDim, kStateDim) - P_priori_ * H_T_ * S_H_) * P_priori_;

    // update com position
    pin_interface_->calcComState(robot_state.pos_gen, robot_state.vel_gen, robot_state.pos_com, robot_state.vel_com);

    // foot

    // std::cout << "===== Robot State2 =====" << std::endl;

    // // // Body state
    std::cout << "Body Position: " << robot_state.body.pos.transpose() << std::endl;
    // std::cout << "Body Quaternion: " << robot_state.body.quat.coeffs().transpose() << std::endl;
    // std::cout << "Body Velocity: " << robot_state.body.vel.transpose() << std::endl;
    // std::cout << "Body Angular Velocity: " << robot_state.body.angvel.transpose() << std::endl;

    // // Generalized State
    // std::cout << "Generalized Position: " << robot_state.pos_gen.transpose() << std::endl;
    // std::cout << "Generalized Velocity: " << robot_state.vel_gen.transpose() << std::endl;

    // // Center of Mass references
    // std::cout << "CoM Position: " << robot_state.pos_com.transpose() << std::endl;
    // std::cout << "CoM Velocity: " << robot_state.vel_com.transpose() << std::endl;
}

void KalmanFilterEstimator::updateCovarianceMatrix(const Vector4i &contact_flag)
{
    for (int i = 0; i < kFeetNum; i++)
    {
        if (contact_flag[i] == false)
        {
            Q_.block<3, 3>(6 + 3 * i, 6 + 3 * i) = kLargeVariance_ * Q_init_.block<3, 3>(6 + 3 * i, 6 + 3 * i);
            R_.block<3, 3>(0 + 3 * i, 0 + 3 * i) = kLargeVariance_ * R_init_.block<3, 3>(0 + 3 * i, 0 + 3 * i);
            R_.block<3, 3>(kFeetDim + 3 * i, kFeetDim + 3 * i) = kLargeVariance_ * R_init_.block<3, 3>(kFeetDim + 3 * i, kFeetDim + 3 * i);
            R_(2 * kFeetDim + i, 2 * kFeetDim + i) = kLargeVariance_ * R_init_(2 * kFeetDim + i, 2 * kFeetDim + i);
        }
        else
        {
            Q_.block<3, 3>(6 + 3 * i, 6 + 3 * i) = Q_init_.block<3, 3>(6 + 3 * i, 6 + 3 * i);
            R_.block<3, 3>(0 + 3 * i, 0 + 3 * i) = R_init_.block<3, 3>(0 + 3 * i, 0 + 3 * i);
            R_.block<3, 3>(kFeetDim + 3 * i, kFeetDim + 3 * i) = R_init_.block<3, 3>(kFeetDim + 3 * i, kFeetDim + 3 * i);
            R_(2 * kFeetDim + i, 2 * kFeetDim + i) = R_init_(2 * kFeetDim + i, 2 * kFeetDim + i);
        }
    }
}

void KalmanFilterEstimator::updateMeasurement(RobotState &robot_state)
{
    pin_interface_->updateKinematics(robot_state.pos_gen, robot_state.vel_gen);
    auto &foot_state = robot_state.foot;
    for (size_t i = 0; i < 4; i++)
    {
        foot_state.pos.col(i) = pin_interface_->getFootPosition(i);
        foot_state.vel.col(i) = pin_interface_->getFootVelocity(i);
        foot_state.pos_rel_body.col(i) = foot_state.pos.col(i) - robot_state.body.pos;
        foot_state.vel_rel_body.col(i) = foot_state.vel.col(i) - robot_state.body.vel;
    }

    for (int i = 0; i < kFeetNum; i++)
    {
        z_.segment<3>(0 + 3 * i) = -foot_state.pos_rel_body.col(i);
        z_.segment<3>(0 + 3 * i)[2] += foot_radius_;
        z_.segment<3>(12 + 3 * i) = -foot_state.vel_rel_body.col(i);
        z_.segment<4>(24) = kFeetHeight_;
    }
}
