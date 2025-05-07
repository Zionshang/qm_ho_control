#include "pinocchio_interface.hpp"

PinocchioInterface::PinocchioInterface()
{
    std::string urdf_filename = getProjectPath() + "/urdf/galileo_v1d6_x5.urdf";

    pin::urdf::buildModel(urdf_filename, pin::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    nq_ = model_.nq;
    nv_ = model_.nv;

    // todo: 用参数指定
    feet_id_.push_back(model_.getFrameId("FL_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("FR_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("RL_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("RR_foot", pinocchio::BODY));

    body_id_ = model_.getFrameId("base_link", pinocchio::BODY);
    mass_ = pinocchio::computeTotalMass(model_);
    std::cout << "total mass of robot: " << mass_ << std::endl;
}

void PinocchioInterface::calcComState(const VectorXd &q, const VectorXd &v, Vector3d &pos_com, Vector3d &vel_com)
{
    pin::centerOfMass(model_, data_, q, v, false);
    pos_com = data_.com[0];
    vel_com = data_.vcom[0];
}

void PinocchioInterface::updateKinematics(const VectorXd &q, const VectorXd &v)
{
    pin::forwardKinematics(model_, data_, q, v);
    pin::updateFramePlacements(model_, data_);
}

Vector3d PinocchioInterface::getFootPosition(int leg_id)
{
    return data_.oMf[feet_id_[leg_id]].translation();
}

Vector3d PinocchioInterface::getFootVelocity(int leg_id)
{
    pin::Motion motion = pin::getFrameVelocity(model_, data_, feet_id_[leg_id], pinocchio::LOCAL_WORLD_ALIGNED);
    return motion.linear();
}

void PinocchioInterface::calcZeroAccKinematics(const VectorXd &q, const VectorXd &v)
{
    pinocchio::forwardKinematics(model_, data_, q, v, VectorXd::Zero(nv_));
}

void PinocchioInterface::calcFootJacobian(const VectorXd &q, int leg_id, Matrix6xd &J)
{
    J.setZero(); // todo: 是否可以删除？
    pinocchio::computeFrameJacobian(model_, data_, q, feet_id_[leg_id], pinocchio::LOCAL_WORLD_ALIGNED, J);
}

void PinocchioInterface::calcBodyJacobian(const VectorXd &q, Matrix6xd &J)
{
    J.setZero(); // todo: 是否可以删除？
    pinocchio::computeFrameJacobian(model_, data_, q, body_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
}

void PinocchioInterface::calcComJacobian(const VectorXd &q, Matrix3xd &J)
{
    J.setZero(); // todo: 是否可以删除？
    pinocchio::jacobianCenterOfMass(model_, data_, q, false);
    J = data_.Jcom;
}

void PinocchioInterface::calcFootJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, int leg_id, Vector6d &dJdq)
{
    dJdq = getFrameClassicalAcceleration(model_, data_, feet_id_[leg_id], pinocchio::LOCAL_WORLD_ALIGNED);
}

void PinocchioInterface::calcBodyJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, Vector6d &dJdq)
{
    dJdq = getFrameClassicalAcceleration(model_, data_, body_id_, pinocchio::LOCAL_WORLD_ALIGNED);
}

void PinocchioInterface::calcComJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, Vector3d &dJdq)
{
    dJdq = data_.acom[0];
}

void PinocchioInterface::calcDynamicsMatrix(const VectorXd &q, const VectorXd &v, MatrixXd &M, VectorXd &C)
{
    pinocchio::crba(model_, data_, q);
    pinocchio::nonLinearEffects(model_, data_, q, v);
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    M = data_.M;
    C = data_.nle;
}
