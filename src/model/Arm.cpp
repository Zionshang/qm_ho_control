#include "model/Arm.h"
#include "iostream"

Arm::Arm()
{
    std::string urdf_filename = getProjectPath() + "/urdf/z1_newGrip.urdf";
    pinocchio::urdf::buildModel(urdf_filename, _model);
    _data = pinocchio::Data(_model);
    _idG = _model.getFrameId("gripperVirtual", pinocchio::BODY);
}

void Arm::setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG)
{
    pinocchio::forwardKinematics(_model, _data, q);
    updateFramePlacement(_model, _data, _idG);

    posG = _data.oMf[_idG].translation();

    pinocchio::SE3::Quaternion quat(_data.oMf[_idG].rotation());
    quatG = quat.coeffs();
}

/**
 * @brief calculate all state (postion, quaternion, velocity, angular velocity) of Gripper
 * @param {Vec6} &q: current joint position of arm
 * @param {Vec6} &dq: current joint velocity of arm
 * @param {Vec3} &posG: position of GRIPER w.r.t. SHOULDER, expressed in SHOULDER frame
 * @param {Quat} &quatG: quaternion of GRIPER frame w.r.t. SHOULDER frame
 * @param {Vec3} &velG: velocity of GRIPER, expressed in SHOULDER frame
 * @param {Vec3} &angVelG: angular velocity of GRIPER, expressed in SHOULDER frame
 */
void Arm::setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG,
                            const Vec6 &dq, Vec3 &velG, Vec3 &angVelG)
{
    pinocchio::forwardKinematics(_model, _data, q, dq);
    updateFramePlacement(_model, _data, _idG);

    posG = _data.oMf[_idG].translation();

    pinocchio::SE3::Quaternion quat(_data.oMf[_idG].rotation());
    quatG = quat.coeffs();

    pinocchio::Motion motion = pinocchio::getFrameVelocity(_model, _data, _idG, pinocchio::LOCAL_WORLD_ALIGNED);
    velG = motion.linear();
    angVelG = motion.angular();
}
