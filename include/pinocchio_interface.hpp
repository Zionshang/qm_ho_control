#pragma once

#include "utils/project_path.hpp"
#include "common/types.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
namespace pin = pinocchio;

class PinocchioInterface
{
public:
    PinocchioInterface();
    const pin::Model &model() const { return model_; }
    const pin::Data &data() const { return data_; }
    pin::Data &mutable_data() { return data_; }

    /**
     * Calculate the position and velocity of whole body center of mass
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     * @param [out] pos_com : position of CoM, expressed in WORLD frame
     * @param [out] vel_com : velocity of CoM, expressed in WORLD frame
     */
    void calcComState(const VectorXd &q, const VectorXd &v, Vector3d &pos_com, Vector3d &vel_com);

    /**
     * Update whole body kinematics
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     */
    void updateKinematics(const VectorXd &q, const VectorXd &v);

    /**
     * Get the position of foot
     * @param [in] leg_id : index of leg. Can only be 0,1,2,3.
     * @warning updateKinematics() should have been called first.
     */
    Vector3d getFootPosition(int leg_id);

    /**
     * Get the velocity of foot
     * @param [in] leg_id : index of leg. Can only be 0,1,2,3.
     * @warning updateKinematics() should have been called first.
     */
    Vector3d getFootVelocity(int leg_id);

    /**
     * Calculate whole body Jacobian of the foot
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] leg_id : index of leg. Can only be 0,1,2,3.
     * @param [out] J : Jacobian matrix. dim:[6 x nv]. This jacobian maps q to velocity of foot, expressed in WORLD frame
     */
    void calcFootJacobian(const VectorXd &q, int leg_id, Matrix6xd &J);

    /**
     * Calculate whole body Jacobian of the body.
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [out] J : Jacobian matrix. dim:[6 x nv]. This jacobian maps q to velocity of body, expressed in WORLD frame
     */
    void calcBodyJacobian(const VectorXd &q, Matrix6xd &J);

    /**
     * Calculate whole body com of the body.
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [out] J : Jacobian matrix. dim:[3 x nv]. This jacobian maps q to linear velocity of com, expressed in WORLD frame
     */
    void calcComJacobian(const VectorXd &q, Matrix3xd &J);

    /**
     * Calculate the product of foot Jacobian's derivative and generalized velocity. dim:[6 x 1]
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     * @param [in] leg_id : index of leg. Can only be 0,1,2,3.
     * @param [out] dJdq : product of Jacobian's derivative and velocity (dJ * dq), which is used in "a = J * ddq + dJ * dq"
     * @warning calcZeroAccKinematics() should have been called first.
     */
    void calcFootJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, int leg_id, Vector6d &dJdq);

    /**
     * Calculate the product of body Jacobian's derivative and generalized velocity. dim:[6 x 1]
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     * @param [out] dJdq : product of Jacobian's derivative and velocity (dJ * dq), which is used in "a = J * ddq + dJ * dq"
     * @warning calcZeroAccKinematics() should have been called first.
     */
    void calcBodyJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, Vector6d &dJdq);

    /**
     * Calculate the product of com Jacobian's derivative and generalized velocity. dim:[6 x 1]
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     * @param [out] dJdq : product of Jacobian's derivative and velocity (dJ * dq), which is used in "a = J * ddq + dJ * dq"
     * @warning calcZeroAccKinematics() should have been called first.
     * todo: 是否需要@warning
     */
    void calcComJacobianTimesVelocity(const VectorXd &q, const VectorXd &v, Vector3d &dJdq);

    /**
     * Calculate the dynamics matrix in rigid-body dynamics equation [M * ddq + C = tau]
     * @param [in] q : generalized position. dim:[nq x 1]
     * @param [in] v : generalized velocity. dim:[nq x 1]
     * @param [out] M : Mass matrix (M) in [M * ddq + C = tau]
     * @param [out] C : Corriolis, centrifual and gravitationnal matrix (C) in [M * ddq + C = tau]
     */
    void calcDynamicsMatrix(const VectorXd &q, const VectorXd &v, MatrixXd &M, VectorXd &C);

    // todo: 是否可以删除？
    void calcZeroAccKinematics(const VectorXd &q, const VectorXd &v);

    int nq() { return nq_; }
    int nv() { return nv_; }
    int body_id() const { return body_id_; }
    const std::vector<int> &feet_id() const { return feet_id_; }

private:
    pin::Model model_;
    pin::Data data_;

    int body_id_;
    std::vector<int> feet_id_;

    int nq_, nv_;
    double mass_;
};
