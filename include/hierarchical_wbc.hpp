#pragma once
#include <vector>
#include <sys/time.h>
#include "common/mathTools.h"
#include "common/HighCmd.h"
#include "common/projectPath.h"
#include "model/WholeBodyDynamics.h"
#include "HoQp.h"
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
    double _fzMin, _fzMax;                // limitation of contact force in z direction
    double _fricRatio;                    // static friction ratio
    Eigen::Matrix<double, 6, 3> _fricMat; // Friction cone matrix  _fricMat * F <= _fricVec
    Vector6d _fricVec;                        // Friction cone vector  _fricMat * F <= _fricVec

    int _nv;
    int _dimDecisionVars;
    VectorXd _idSw;     // stores the ID of swing leg
    VectorXd _solFinal; // solution of fianl hierarchical optimization

    int _nSt, _nSw; // number of stance and swing leg
    MatrixXd _M;       // Matrix M in M*ddq + C = tau
    VecNv _C;       // Matrix C in M*ddq + C = tau
    // TODO: 是否需要使用std::Vector
    std::vector<Matrix6xd> _Jfeet = std::vector<Matrix6xd>(4); // jacobian of all feet
    std::vector<Vector6d> _dJdqfeet = std::vector<Vector6d>(4);        // jacobian's derviative times v of all feet
    Eigen::Matrix<double, -1, ROBOTNV> _Jst, _Jsw;             // jacobian of stacked swing and stance feet
    Eigen::VectorXd _dJdqst, _dJdqsw;                          // jacobian's derviative times v of stacked swing and stance feet
    Matrix6xd _Jbody;                                          // jacobian of body
    Vector6d _dJdqbody;                                            // jacobian's derviative times v of body
    Matrix3xd _Jcom;                                           // jacobian of CoM
    Vector3d _dJdqcom;                                             // jacobian's derviative times v of CoM
    Vector3d _posCoM;                                              // position of full body CoM
    Vector3d _velCoM;                                              // velocity of full body CoM

    // PD parameter
    Diagonal3d _KpPos, _KdPos; // PD parameter of body position control
    Diagonal3d _KpAng, _KdAng; // PD parameter of body angular control
    Diagonal3d _KpSw, _KdSw;   // PD parameter of swing leg control

    // weight of QP
    double _wBlinear, _wBangle, _wSw, _wElinear, _wEangle, _wArmJ;

    // Matrix6xd _Jgrip; // jacobian of gripper
    // Vector6d _dJdqgrip;   // jacobian's derviative times v of gripper

    Diagonal3d _KpEePos, _KdEePos; // PD parameter of EE position control
    Diagonal3d _KpEeAng, _KdEeAng; // PD parameter of EE angular control
    Diagonal6d _KpArmJ, _KdArmJ;   // PD parameter of arm joing
};
