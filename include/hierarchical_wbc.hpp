#pragma once
#include <vector>
#include <sys/time.h>
#include "common/mathTools.h"
#include "common/HighCmd.h"
#include "common/projectPath.h"
#include "model/WholeBodyDynamics.h"
#include "hierarchical_qp.hpp"
#include "Estimator.h"
#include "yaml-cpp/yaml.h"
#include "Task.h"

class HierarchicalWbc
{

public:
    HierarchicalWbc(HighCmd *highCmd, WholeBodyDynamics *wbDyn);
    ~HierarchicalWbc();

    void calTau(const RobotState &robot_state, const Vector4i contact, VectorXd &tau);

private:
    void paramInit(std::string fileName);
    void updateJacobian(const VectorXd &pos_gen, const VectorXd &vel_gen, const Vector4i &contact);

    Task buildFloatingBaseEomTask();
    Task buildNoContactMotionTask();
    Task buildFrictionConeTask();
    Task buildBodyLinearTask(const Vector3d &pos_body, const Vector3d &vel_body,
                             const Vector3d &pos_body_ref, const Vector3d &vel_body_ref);
    Task buildBodyAngularTask(const Quaternion &quat_body, const Vector3d &angvel_body,
                              const Quaternion &quat_body_ref, const Vector3d &angvel_body_ref);
    Task buildComLinearTask(const Vector3d &pos_com, const Vector3d &vel_com,
                            const Vector3d &pos_com_ref, const Vector3d &vel_com_ref);
    Task buildSwingLegTask(const Matrix34d &pos_feet, const Matrix34d &vel_feet,
                           const Matrix34d &pos_feet_ref, const Matrix34d &vel_feet_ref);
    Task buildArmJointTask(const Vector6d &pos_arm, const Vector6d &vel_arm,
                           const Vector6d &pos_arm_ref, const Vector6d &vel_arm_ref);
    // Task buildGripperLinearTask();
    // Task buildGripperAngularTask();

    WholeBodyDynamics *_wbDyn;
    HighCmd *_highCmd;

    // friction cone
    double fz_min_, fz_max_;               // limitation of contact force in z direction
    double fric_coef_;                     // static friction ratio
    Eigen::Matrix<double, 6, 3> fric_mat_; // Friction cone matrix  fric_mat_ * F <= fric_vec_
    Vector6d fric_vec_;                    // Friction cone vector  fric_mat_ * F <= fric_vec_

    int nv_;                // dimension of generalized velocity
    int dim_decision_vars_; // dimension of decision variables
    VectorXd id_sw_;        // stores the ID of swing leg
    VectorXd sol_final_;    // solution of fianl hierarchical optimization

    int num_st_, num_sw_; // number of stance and swing leg
    MatrixXd M_;          // Matrix M in M*ddq + C = tau
    VecNv C_;             // Matrix C in M*ddq + C = tau

    // TODO: 是否需要使用std::Vector
    std::vector<Matrix6xd> J_feet_ = std::vector<Matrix6xd>(4);  // jacobian of all feet
    std::vector<Vector6d> dJdq_feet_ = std::vector<Vector6d>(4); // jacobian's derviative times v of all feet
    Eigen::Matrix<double, -1, ROBOTNV> J_st_, J_sw_;             // jacobian of stacked swing and stance feet
    VectorXd dJdq_st_;                                           // jacobian's derviative times v of stacked stance feet
    VectorXd dJdq_sw_;                                           // jacobian's derviative times v of stacked swing feet
    Matrix6xd J_body_;                                           // jacobian of body
    Vector6d dJdq_body_;                                         // jacobian's derviative times v of body
    Matrix3xd J_com_;                                            // jacobian of CoM
    Vector3d dJdq_com_;                                          // jacobian's derviative times v of CoM

    Diagonal3d kp_pos_, kd_pos_; // PD parameter of body position control
    Diagonal3d kp_ang_, kd_ang_; // PD parameter of body angular control
    Diagonal3d kp_sw_, kd_sw_;   // PD parameter of swing leg control
    Diagonal6d kp_arm_, kd_arm_; // PD parameter of arm joint

    double weight_pos_body_, weight_pos_ang_; // weight of body task
    double weight_swing_;                     // weight of swing leg task
    double weight_arm_;                       // weight of arm joint task

    // Matrix6xd J_grip_; // jacobian of gripper
    // Vector6d dJdq_grip_;   // jacobian's derviative times v of gripper
    // double weight_gripper_pos_, weight_gripper_ang_; // weight of gripper task
    // Diagonal3d _KpEePos, _KdEePos; // PD parameter of EE position control
    // Diagonal3d _KpEeAng, _KdEeAng; // PD parameter of EE angular control
};
