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

class HoControl
{

public:
    HoControl(HighCmd *highCmd, Estimator *est, WholeBodyDynamics *wbDyn);
    ~HoControl();

    void calTau(const Vector4i contact, Vec18 &tau);

private:
    void paramInit(std::string fileName);
    void initVars(const Vector4i &contact);
    Task buildFloatingBaseEomTask();
    Task buildNoContactMotionTask();
    Task buildFrictionConeTask();
    Task buildBodyLinearTask();
    Task buildBodyAngularTask();
    Task buildBodyAccTask();
    Task buildSwingLegTask();
    Task buildGripperLinearTask();
    Task buildGripperAngularTask();
    Task buildArmJointTrackingTask();

    WholeBodyDynamics *_wbDyn;
    LowState *_lowState;
    HighCmd *_highCmd;
    Estimator *_est;

    // friction cone
    double _fzMin, _fzMax;                // limitation of contact force in z direction
    double _fricRatio;                    // static friction ratio
    Eigen::Matrix<double, 6, 3> _fricMat; // Friction cone matrix  _fricMat * F <= _fricVec
    Vec6 _fricVec;                        // Friction cone vector  _fricMat * F <= _fricVec

    int _nv;
    int _dimDecisionVars;
    VecX _idSw;     // stores the ID of swing leg
    VecX _solFinal; // solution of fianl hierarchical optimization

    int _nSt, _nSw;                                     // number of stance and swing leg
    RotMat _R;                                          // rotation of BODY reletive to GLOBAL
    VecNq _q;                                           // Generalized position [body position; body quaternion; actuated joint position], expressed in Global frame
    VecNv _v;                                           // Generalized velocity [body velocity; body anguler velocity; actuated joint position], expressed in BODY frame
    MatNv _M;                                           // Matrix M in M*ddq + C = tau
    VecNv _C;                                           // Matrix C in M*ddq + C = tau
    std::vector<Jacb> _Jfeet = std::vector<Jacb>(4);    // jacobian of all feet
    std::vector<Vec6> _dJdqfeet = std::vector<Vec6>(4); // jacobian's derviative times v of all feet
    Eigen::Matrix<double, -1, ROBOTNV> _Jst, _Jsw;      // jacobian of stacked swing and stance feet
    Eigen::VectorXd _dJdqst, _dJdqsw;                   // jacobian's derviative times v of stacked swing and stance feet
    Jacb _Jbody;                                        // jacobian of body
    Vec6 _dJdqbody;                                     // jacobian's derviative times v of body
    JacbP _Jcom;                                        // jacobian of CoM
    Vec3 _dJdqcom;                                      // jacobian's derviative times v of CoM
    Vec3 _posCoM;                                       // position of full body CoM
    Vec3 _velCoM;                                       // velocity of full body CoM

    // PD parameter
    Diag3 _KpPos, _KdPos; // PD parameter of body position control
    Diag3 _KpAng, _KdAng; // PD parameter of body angular control
    Diag3 _KpSw, _KdSw;   // PD parameter of swing leg control

    // weight of QP
    double _wBlinear, _wBangle, _wSw, _wElinear, _wEangle, _wArmJ;

    Jacb _Jgrip;    // jacobian of gripper
    Vec6 _dJdqgrip; // jacobian's derviative times v of gripper

    Diag3 _KpEePos, _KdEePos; // PD parameter of EE position control
    Diag3 _KpEeAng, _KdEeAng; // PD parameter of EE angular control
    Diag6 _KpArmJ, _KdArmJ;   // PD parameter of arm joing
};
