#include "model/WholeBodyDynamics.h"

WholeBodyDynamics::WholeBodyDynamics()
{
    std::string urdfName = getProjectPath() + "/urdf/aliengoZ1_newGrip.urdf";

    pinocchio::urdf::buildModel(urdfName, _model);
    _data = pinocchio::Data(_model);
    _nq = _model.nq;
    _nv = _model.nv;

    _idFeet[0] = _model.getFrameId("leg1_foot", pinocchio::BODY);
    _idFeet[1] = _model.getFrameId("leg2_foot", pinocchio::BODY);
    _idFeet[2] = _model.getFrameId("leg3_foot", pinocchio::BODY);
    _idFeet[3] = _model.getFrameId("leg4_foot", pinocchio::BODY);
    _idBody = _model.getFrameId("trunk", pinocchio::BODY);
    _idGripper = _model.getFrameId("gripperVirtual", pinocchio::BODY);
}

void WholeBodyDynamics::calcZeroAccKinematics(const VecNq &q, const VecNv &v)
{
    VecNv a;
    a.setZero();
    pinocchio::forwardKinematics(_model, _data, q, v, a);
}

/**
 * @brief Calculate and set the position and velocity of whole body center of mass
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {Vector3d} &posCoM: position of CoM, expressed in WORLD frame
 * @param {Vector3d} &velCoM: velocity of CoM, expressed in WORLD frame
 */
void WholeBodyDynamics::setCoMPosVel(const VecNq &q, const VecNv &v, Vector3d &posCoM, Vector3d &velCoM)
{
    pinocchio::centerOfMass(_model, _data, q, v, false);
    posCoM = _data.com[0];
    velCoM = _data.vcom[0];
}

/**
 * @brief Calculate and set whole body Jacobian of the foot.
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {int} idLeg: Can only be 0,1,2,3
 * @param {Matrix6xd} Jacobian to be set. dim:[6 x nv]
 * @note This jacobian maps q to velocity of FOOT, expressed in WORLD frame
 */
void WholeBodyDynamics::setFootJacob(const VecNq &q, int idLeg, Matrix6xd &J)
{
    J.setZero();
    pinocchio::computeFrameJacobian(_model, _data, q, _idFeet[idLeg], pinocchio::LOCAL_WORLD_ALIGNED, J);
}

/**
 * @brief: Calculate and set whole body Jacobian's derivative times generalized velocity of foot. dim:[6 x 1]
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {int} idLeg: Can only be 0,1,2,3
 * @param {Vector6d} &dJdq: Jacobian's derivative times v, which is in "a = J * ddq + dJ * dq"
 * @note You must call WholeBodyDynamics::updateKinematics() in advance
 */
void WholeBodyDynamics::setFootdJdq(const VecNq &q, const VecNv &v, int idLeg, Vector6d &dJdq)
{
    // VecNv a;
    // a.fill(0.);
    // pinocchio::forwardKinematics(_model, _data, q, v, a);
    dJdq = getFrameClassicalAcceleration(_model, _data, _idFeet[idLeg], pinocchio::LOCAL_WORLD_ALIGNED);
}

/**
 * @brief Calculate and set M and C term which is in dynamic equation [M * ddq + C = tau]
 * @param {VecNq} &q: Generalized position. dim:[nv x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {MatNv} &M: The M to be set
 * @param {VecNv} &C: The C to be set
 */
void WholeBodyDynamics::setMandC(const VecNq &q, const VecNv &v, MatrixXd &M, VecNv &C)
{
    pinocchio::crba(_model, _data, q);
    pinocchio::nonLinearEffects(_model, _data, q, v);
    M.triangularView<Eigen::Upper>() = _data.M;
    M.triangularView<Eigen::Lower>() = M.triangularView<Eigen::Upper>().transpose();
    C = _data.nle;
}

/**
 * @brief Calculate and set whole body Jacobian of the body.
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {Matrix6xd} &J: Jacobian to be set. dim:[6 x nv]
 * @note This jacobian maps q to velocity of BODY, expressed in WORLD frame
 */
void WholeBodyDynamics::setBodyJacob(const VecNq &q, Matrix6xd &J)
{
    J.setZero();
    pinocchio::computeFrameJacobian(_model, _data, q, _idBody, pinocchio::LOCAL_WORLD_ALIGNED, J);
}

/**
 * @brief Calculate and set whole body Jacobian's derivative times generalized velocity of BODY. dim:[6 x 1]
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {Vector6d} &dJdq: Jacobian's derivative times v, which is in "a = J * ddq + dJ * dq"
 * @note You must call WholeBodyDynamics::updateKinematics() in advance
 */
void WholeBodyDynamics::setBodydJdq(const VecNq &q, const VecNv &v, Vector6d &dJdq)
{
    // VecNv a;
    // a.fill(0.);
    // pinocchio::forwardKinematics(_model, _data, q, v, a);
    dJdq = getFrameClassicalAcceleration(_model, _data, _idBody, pinocchio::LOCAL_WORLD_ALIGNED);
}

/**
 * @brief Calculate and set whole body Jacobian of the CoM.
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {Matrix6xd} &J: Jacobian to be set. dim:[6 x nv]
 * @note This jacobian maps q to velocity of COM, expressed in WORLD frame
 */
void WholeBodyDynamics::setCoMJacob(const VecNq &q, Matrix3xd &J)
{
    J.setZero();
    pinocchio::jacobianCenterOfMass(_model, _data, q, false);
    J = _data.Jcom;
}

/**
 * @brief Calculate and set whole body Jacobian's derivative times generalized velocity of COM. dim:[6 x 1]
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {Vector6d} &dJdq: Jacobian's derivative times v, which is in "a = J * ddq + dJ * dq"
 */
void WholeBodyDynamics::setCoMdJdq(const VecNq &q, const VecNv &v, Vector3d &dJdq)
{
    // VecNv a;
    // a.fill(0.);
    // pinocchio::centerOfMass(_model, _data, q, v, a, false);
    dJdq = _data.acom[0];
}

/**
 * @brief Calculate and set whole body Jacobian of the GRIPPER.
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {Matrix6xd} &J: Jacobian to be set. dim:[6 x nv]
 * @note This jacobian maps q to velocity of GRIPPER, expressed in WORLD frame
 */
void WholeBodyDynamics::setGripperJacob(const VecNq &q, Matrix6xd &J)
{
    J.setZero();
    pinocchio::computeFrameJacobian(_model, _data, q, _idGripper, pinocchio::LOCAL_WORLD_ALIGNED, J);
}

/**
 * @brief Calculate and set whole body Jacobian's derivative times generalized velocity of gripper. dim:[6 x 1]
 * @param {VecNq} &q: Generalized position. dim:[nq x 1]
 * @param {VecNv} &v: Generalized velocity. dim:[nv x 1]
 * @param {Vector6d} &dJdq: Jacobian's derivative times v, which is in "a = J * ddq + dJ * dq"
 * @note You must call WholeBodyDynamics::updateKinematics() in advance
 */
void WholeBodyDynamics::setGripperdJdq(const VecNq &q, const VecNv &v, Vector6d &dJdq)
{
    // VecNv a;
    // a.fill(0.);
    // pinocchio::forwardKinematics(_model, _data, q, v, a);
    dJdq = getFrameClassicalAcceleration(_model, _data, _idGripper, pinocchio::LOCAL_WORLD_ALIGNED);
}
