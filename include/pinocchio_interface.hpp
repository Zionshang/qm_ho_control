#pragma once

#include "common/projectPath.h"
#include "common/mathTypes.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>

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

    void updateKinematics(const VectorXd &q, const VectorXd &v);

    Vector3d calcFootPosition(int foot_id);
    Vector3d calcFootVelocity(int foot_id);

    void calcGripperState(Vector3d &pos_gripper, Vector3d &vel_gripper,
                          Quaternion &quat_gripper, Vector3d &angvel_gripper);

    int nq() { return nq_; }
    int nv() { return nv_; }
    int body_id() const { return body_id_; }
    int gripper_id() const { return gripper_id_; }
    const std::vector<int> &feet_id() const { return feet_id_; }

private:
    pin::Model model_;
    pin::Data data_;

    int body_id_;
    int gripper_id_;
    std::vector<int> feet_id_;

    int nq_, nv_;
};
