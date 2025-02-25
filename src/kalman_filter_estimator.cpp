#include "kalman_filter_estimator.hpp"

KalmanFilterEstimator::KalmanFilterEstimator(LowState *lowState, PinocchioInterface *pin_interface, double dt)
    : low_state_(lowState), pin_interface_(pin_interface)
{
    // 初始化广义向量
    pos_gen_.setZero(pin_interface_->nq());
    vel_gen_.setZero(pin_interface_->nv());

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
    Q_init_.block<3, 3>(0, 0) = (dt / 20.) * kIdentity3_ * noise_processimu__position_;
    Q_init_.block<3, 3>(3, 3) = (dt * 9.81 / 20.) * kIdentity3_ * noise_processimu__velocity_;
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
void KalmanFilterEstimator::update(const Vector4i &contact_flag)
{
    pos_body_ = x_hat_.segment<3>(0);
    vel_body_ = x_hat_.segment<3>(3);

    quat_body_ = low_state_->getQuaternion();
    rotmat_body_ = quat_body_.toRotationMatrix();
    angvel_body_B_ = low_state_->getGyro();

    pos_leg_ = low_state_->getLegJointPosition();
    vel_leg_ = low_state_->getLegJointVelocity();
    pos_arm_ = low_state_->getArmJointPosition();
    vel_arm_ = low_state_->getArmJointVelocity();

    // update covariance matix because of swing leg
    updateCovarianceMatrix(contact_flag);

    // update measurement
    pos_gen_ << pos_body_, quat_body_, mat34ToVec12(pos_leg_), pos_arm_;
    vel_gen_ << rotmat_body_.transpose() * vel_body_, angvel_body_B_, mat34ToVec12(vel_leg_), vel_arm_;
    updateMeasurement();

    // update state transition
    Vector3d acc_body = rotmat_body_ * low_state_->getAccelerometer() + kGravity_;
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
    pin_interface_->calcComState(pos_gen_, vel_gen_, pos_com_, vel_com_);

    std::cout << std::left << "KLF Body Position:     \t"
              << pos_body_.transpose() << std::endl;
    std::cout << std::left << "KLF Body Velocity:     \t"
              << vel_body_.transpose() << std::endl;
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

void KalmanFilterEstimator::updateMeasurement()
{
    pin_interface_->updateKinematics(pos_gen_, vel_gen_);
    for (size_t i = 0; i < 4; i++)
    {
        pos_feet_.col(i) = pin_interface_->getFootPosition(i);
        vel_feet_.col(i) = pin_interface_->getFootVelocity(i);
        pos_feet_rel_body.col(i) = pos_feet_.col(i) - pos_body_;
        vel_feet_rel_body.col(i) = vel_feet_.col(i) - vel_body_;
    }

    for (int i = 0; i < kFeetNum; i++)
    {
        z_.segment<3>(0 + 3 * i) = -pos_feet_rel_body.col(i);
        z_.segment<3>(0 + 3 * i)[2] += foot_radius_;
        z_.segment<3>(12 + 3 * i) = -vel_feet_rel_body.col(i);
        z_.segment<4>(24) = kFeetHeight_;
    }
}
