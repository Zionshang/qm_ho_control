#include "hierarchical_wbc.hpp"

HierarchicalWbc::HierarchicalWbc(HighCmd *highCmd, Estimator *est, WholeBodyDynamics *wbDyn)
    : _highCmd(highCmd), _est(est), _wbDyn(wbDyn)
{
    _nv = _wbDyn->getNv();
    _fricRatio = 0.7;

    // friction cone constraint    _fricMat * [Fx Fy Fz]^T <= _fricVec
    _fzMax = 2000;
    _fzMin = 10;
    _fricMat << 0, 0, 1,
        0, 0, -1,
        1, 0, -_fricRatio,
        0, 1, -_fricRatio,
        -1, 0, -_fricRatio,
        0, -1, -_fricRatio;
    _fricVec << _fzMax, -_fzMin, 0, 0, 0, 0;

    // PD paramerter
    std::string configFile = getProjectPath() + "/config/config.yml";
    paramInit(configFile);
}

HierarchicalWbc::~HierarchicalWbc()
{
    delete _wbDyn;
}

void HierarchicalWbc::calTau(const RobotState &robot_state, const Vector4i contact, Vec18 &tau)
{
    // struct timeval t1, t2;
    // gettimeofday(&t1, NULL);
    // gettimeofday(&t2, NULL);
    // std::cout << "Time of setMatrix: \t " << double(t2.tv_usec - t1.tv_usec) / 1000 << "ms" << std::endl;

    const auto &q_gen = robot_state.pos_gen;
    const auto &v_gen = robot_state.vel_gen;
    const auto &pos_body = robot_state.body.pos;
    const auto &vel_body = robot_state.body.vel;
    const auto &quat_body = robot_state.body.quat;
    const auto &angvel_body = robot_state.body.angvel;
    const auto &pos_com = robot_state.pos_com;
    const auto &vel_com = robot_state.vel_com;
    const auto &pos_feet = robot_state.foot.pos;
    const auto &vel_feet = robot_state.foot.vel;
    const auto &pos_arm = robot_state.joint.pos_arm;
    const auto &vel_arm = robot_state.joint.vel_arm;

    const auto &pos_body_ref = _highCmd->posB;
    const auto &vel_body_ref = _highCmd->velB;
    const auto &quat_body_ref = _highCmd->quatB;
    const auto &angvel_body_ref = _highCmd->angVelB;
    const auto &pos_com_ref = _highCmd->posCoM;
    const auto &vel_com_ref = _highCmd->velCoM;
    const auto &pos_feet_ref = _highCmd->posF;
    const auto &vel_feet_ref = _highCmd->velF;
    const auto &pos_arm_ref = _highCmd->qAJ;
    const auto &vel_arm_ref = _highCmd->dqAJ;

    _dimDecisionVars = _nv + 3 * contact.sum();

    _wbDyn->setMandC(q_gen, v_gen, _M, _C);
    updateJacobian(q_gen, v_gen, contact);

    Task task0, task1, task2;
    task0 = buildFloatingBaseEomTask();
    task0 = task0 + buildNoContactMotionTask();
    task0 = task0 + buildFrictionConeTask();

    task1 = buildComLinearTask(pos_com, vel_com, pos_com_ref, vel_com_ref) * _wBlinear;
    task1 = task1 + buildBodyAngularTask(quat_body, angvel_body, quat_body_ref, angvel_body_ref) * _wBangle;
    task1 = task1 + buildSwingLegTask(pos_feet, vel_feet, pos_feet_ref, vel_feet_ref) * _wSw;
    task1 = task1 + buildArmJointTask(pos_arm, vel_arm, pos_arm_ref, vel_arm_ref) * _wArmJ;

    HoQp hoQp(task1, std::make_shared<HoQp>(task0));
    _solFinal = hoQp.getSolutions();

    tau = _M.bottomRows(_nv - 6) * _solFinal.head(_nv) + _C.bottomRows(_nv - 6) - (_Jst.rightCols(_nv - 6)).transpose() * _solFinal.tail(3 * _nSt);
}

void HierarchicalWbc::updateJacobian(const VectorXd &pos_gen, const VectorXd &vel_gen, const Vector4i &contact)
{
    // Set up zero-acceleration kinematics, used for calculate dJdq.
    // Because a = J * ddq + dJ * dq. When ddq = 0, a = dJ * dq
    _wbDyn->calcZeroAccKinematics(pos_gen, vel_gen);

    // feet Jacobian
    _nSt = contact.sum();
    _nSw = 4 - _nSt;
    _idSw.resize(_nSw);

    int indexSt = 0;
    _Jst.setZero(3 * _nSt, ROBOTNV);
    _dJdqst.setZero(3 * _nSt);

    int indexSw = 0;
    _Jsw.setZero(3 * _nSw, ROBOTNV);
    _dJdqsw.setZero(3 * _nSw);

    for (int i = 0; i < 4; i++)
    {
        _wbDyn->setFootJacob(pos_gen, i, _Jfeet[i]);
        _wbDyn->setFootdJdq(pos_gen, vel_gen, i, _dJdqfeet[i]);

        if (contact(i) == 1)
        {
            _Jst.middleRows(3 * indexSt, 3) = _Jfeet[i].topRows(3);    // only get the linear part
            _dJdqst.segment(3 * indexSt, 3) = _dJdqfeet[i].topRows(3); // only get the linear part
            indexSt++;
        }
        else
        {
            _Jsw.middleRows(3 * indexSw, 3) = _Jfeet[i].topRows(3);    // only get the linear part
            _dJdqsw.segment(3 * indexSw, 3) = _dJdqfeet[i].topRows(3); // only get the linear part
            _idSw(indexSw) = i;
            indexSw++;
        }
    }

    // body Jacobian
    _wbDyn->setBodyJacob(pos_gen, _Jbody);
    _wbDyn->setBodydJdq(pos_gen, vel_gen, _dJdqbody);

    // CoM Jacobian
    _wbDyn->setCoMJacob(pos_gen, _Jcom);
    _wbDyn->setCoMdJdq(pos_gen, vel_gen, _dJdqcom);

    // gripper Jacobian
    _wbDyn->setGripperJacob(pos_gen, _Jgrip);
    _wbDyn->setGripperdJdq(pos_gen, vel_gen, _dJdqgrip);
}

Task HierarchicalWbc::buildFloatingBaseEomTask()
{
    MatX A = (MatX(6, _dimDecisionVars) << _M.topRows(6), -(_Jst.leftCols(6)).transpose()).finished();
    VecX b = -_C.topRows(6);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildNoContactMotionTask()
{
    MatX A = MatX::Zero(3 * _nSt, _dimDecisionVars);
    A.topLeftCorner(3 * _nSt, _nv) = _Jst;
    VecX b = -_dJdqst;

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildFrictionConeTask()
{
    MatX D = MatX::Zero(6 * _nSt, _dimDecisionVars);
    VecX f = VecX(6 * _nSt);
    for (int i = 0; i < _nSt; i++)
    {
        D.block(6 * i, _nv + 3 * i, 6, 3) = _fricMat;
        f.segment(6 * i, 6) = _fricVec;
    }

    return {MatX(), VecX(), D, f};
}

Task HierarchicalWbc::buildBodyLinearTask(const Vector3d &pos_body, const Vector3d &vel_body,
                                          const Vector3d &pos_body_ref, const Vector3d &vel_body_ref)
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    // body linear motion tracking
    A.leftCols(_nv) = _Jbody.topRows(3);
    b = _KpPos * (pos_body_ref - pos_body) + _KdPos * (vel_body_ref - vel_body) - _dJdqbody.head(3);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildComLinearTask(const Vector3d &pos_com, const Vector3d &vel_com,
                                         const Vector3d &pos_com_ref, const Vector3d &vel_com_ref)
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    // CoM linear motion tracking
    A.leftCols(_nv) = _Jcom.topRows(3);
    b = _KpPos * (pos_com_ref - pos_com) + _KdPos * (vel_com_ref - vel_com) - _dJdqcom.head(3);
    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildBodyAngularTask(const Quaternion &quat_body, const Vector3d &angvel_body,
                                           const Quaternion &quat_body_ref, const Vector3d &angvel_body_ref)
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    A.leftCols(_nv) = _Jbody.bottomRows(3);
    b = _KpAng * quatErr(quat_body_ref, quat_body) + _KdAng * (angvel_body_ref - angvel_body) - _dJdqbody.tail(3);
    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildSwingLegTask(const Matrix34d &pos_feet, const Matrix34d &vel_feet,
                                        const Matrix34d &pos_feet_ref, const Matrix34d &vel_feet_ref)
{
    MatX A = MatX::Zero(3 * _nSw, _dimDecisionVars);
    VecX b = VecX(3 * _nSw);

    A.leftCols(_nv) = _Jsw;
    // for (int i = 0; i < _nSw; i++)
    //     b.segment(3 * i, 3) = _KpSw * (_highCmd->posF.col(_idSw(i)) - _est->getPosF().col(_idSw(i))) + _KdSw * (_highCmd->velF.col(_idSw(i)) - _est->getVelF().col(_idSw(i))) - _dJdqsw.segment(3 * i, 3);

    for (int i = 0; i < _nSw; i++)
    {
        b.segment(3 * i, 3) = _KpSw * (pos_feet_ref.col(_idSw(i)) - pos_feet.col(_idSw(i))) +
                              _KdSw * (vel_feet_ref.col(_idSw(i)) - vel_feet.col(_idSw(i))) - _dJdqsw.segment(3 * i, 3);
    }

    return {A, b, MatX(), VecX()};
}

// Task HierarchicalWbc::buildGripperLinearTask()
// {
//     MatX A = MatX::Zero(3, _dimDecisionVars);
//     VecX b = VecX(3);

//     A.leftCols(_nv) = _Jgrip.topRows(3);
//     b = _KpEePos * (_highCmd->posG - _est->getPosG()) + _KdEePos * (_highCmd->velG - _est->getVelG()) - _dJdqgrip.head(3);

//     return {A, b, MatX(), VecX()};
// }

// Task HierarchicalWbc::buildGripperAngularTask()
// {
//     MatX A = MatX::Zero(3, _dimDecisionVars);
//     VecX b = VecX(3);

//     A.leftCols(_nv) = _wEangle * _Jgrip.bottomRows(3);
//     b = _KpEeAng * quatErr(_highCmd->quatG, _est->getQuatG()) + _KdEeAng * (_highCmd->angVelG - _est->getAngVelG()) - _dJdqgrip.tail(3);

//     return {A, b, MatX(), VecX()};
// }

Task HierarchicalWbc::buildArmJointTask(const Vector6d &pos_arm, const Vector6d &vel_arm,
                                        const Vector6d &pos_arm_ref, const Vector6d &vel_arm_ref)
{
    MatX A = MatX::Zero(6, _dimDecisionVars);
    VecX b = VecX(6);

    A.block(0, _nv - 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
    // b = _KpArmJ * (_highCmd->qAJ - _est->getQArm()) + _KdArmJ * (_highCmd->dqAJ - _est->getDqArm());
    b = _KpArmJ * (pos_arm_ref - pos_arm) + _KdArmJ * (vel_arm_ref - vel_arm);

    return {A, b, MatX(), VecX()};
}

void HierarchicalWbc::paramInit(std::string fileName)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(fileName);
    }
    catch (YAML::BadFile &e)
    {
        std::cerr << "Read error in ymal file!" << std::endl;
        exit(-1);
    }

    // PD parameter
    Vec3 kpPos, kdPos, kpAng, kdAng, kpSw, kdSw;
    for (int i = 0; i < 3; i++)
    {
        kpPos(i) = config["kpPos"][i].as<double>();
        kdPos(i) = config["kdPos"][i].as<double>();
        kpAng(i) = config["kpAng"][i].as<double>();
        kdAng(i) = config["kdAng"][i].as<double>();
        kpSw(i) = config["kpSw"][i].as<double>();
        kdSw(i) = config["kdSw"][i].as<double>();
    }

    _KpPos = kpPos.asDiagonal();
    _KdPos = kdPos.asDiagonal();
    _KpAng = kpAng.asDiagonal();
    _KdAng = kdAng.asDiagonal();
    _KpSw = kpSw.asDiagonal();
    _KdSw = kdSw.asDiagonal();

    _wBlinear = config["wBlinear"].as<double>();
    _wBangle = config["wBangle"].as<double>();
    _wSw = config["wSw"].as<double>();

    std::cout << "The PD parameter of Body is: \n";
    std::cout << "kpPos:   \t" << kpPos.transpose() << std::endl;
    std::cout << "kdPos:   \t" << kdPos.transpose() << std::endl;
    std::cout << "kpAng:   \t" << kpAng.transpose() << std::endl;
    std::cout << "kdAng:   \t" << kdAng.transpose() << std::endl;
    std::cout << "kpSw:    \t" << kpSw.transpose() << std::endl;
    std::cout << "kdSw:    \t" << kdSw.transpose() << std::endl;
    std::cout << "The weight of Body is: \n";
    std::cout << "_wBlinear: " << _wBlinear << "\t\t";
    std::cout << "_wBangle : " << _wBangle << "\t\t";
    std::cout << "_wSw     : " << _wSw << std::endl;

    Vec3 kpEePos, kdEePos, kpEeAng, kdEeAng;
    Vec6 kpArmJ, kdArmJ;
    for (int i = 0; i < 3; i++)
    {
        kpEePos(i) = config["kpEePos"][i].as<double>();
        kdEePos(i) = config["kdEePos"][i].as<double>();
        kpEeAng(i) = config["kpEeAng"][i].as<double>();
        kdEeAng(i) = config["kdEeAng"][i].as<double>();
    }
    for (int i = 0; i < 6; i++)
    {
        kpArmJ(i) = config["kpArmJ"][i].as<double>();
        kdArmJ(i) = config["kdArmJ"][i].as<double>();
    }

    _KpEePos = kpEePos.asDiagonal();
    _KdEePos = kdEePos.asDiagonal();
    _KpEeAng = kpEeAng.asDiagonal();
    _KdEeAng = kdEeAng.asDiagonal();
    _KpArmJ = kpArmJ.asDiagonal();
    _KdArmJ = kdArmJ.asDiagonal();

    _wElinear = config["wElinear"].as<double>();
    _wEangle = config["wEangle"].as<double>();
    _wArmJ = config["wArmJ"].as<double>();

    std::cout << "The PD parameter of Arm is: \n";
    std::cout << "kpEePos: \t" << kpEePos.transpose() << std::endl;
    std::cout << "kdEePos: \t" << kdEePos.transpose() << std::endl;
    std::cout << "kpEeAng: \t" << kpEeAng.transpose() << std::endl;
    std::cout << "kdEeAng: \t" << kdEeAng.transpose() << std::endl;
    std::cout << "_KpArmJ: \t" << kpArmJ.transpose() << std::endl;
    std::cout << "_KdArmJ: \t" << kdArmJ.transpose() << std::endl;
    std::cout << "The weight of Arm is: \n";
    std::cout << "wElinear: " << _wElinear << "\t\t";
    std::cout << "WEangle : " << _wEangle << "\t\t";
    std::cout << "_wArmJ  : " << _wArmJ << std::endl;
}
