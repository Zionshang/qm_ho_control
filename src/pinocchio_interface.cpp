#include "pinocchio_interface.hpp"

PinocchioInterface::PinocchioInterface()
{
    std::string urdf_filename = getProjectPath() + "/urdf/aliengoZ1_newGrip.urdf";

    pin::urdf::buildModel(urdf_filename, model_);
    data_ = pinocchio::Data(model_);
    nq_ = model_.nq;
    nv_ = model_.nv;

    feet_id_.push_back(model_.getFrameId("leg1_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("leg2_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("leg3_foot", pinocchio::BODY));
    feet_id_.push_back(model_.getFrameId("leg4_foot", pinocchio::BODY));

    body_id_ = model_.getFrameId("trunk", pinocchio::BODY);
    gripper_id_ = model_.getFrameId("gripperVirtual", pinocchio::BODY);
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

Vector3d PinocchioInterface::calcFootPosition(int foot_id)
{
    return data_.oMf[foot_id].translation();
}

Vector3d PinocchioInterface::calcFootVelocity(int foot_id)
{
    pin::Motion motion = pin::getFrameVelocity(model_, data_, foot_id, pinocchio::LOCAL_WORLD_ALIGNED);
    return motion.linear();
}