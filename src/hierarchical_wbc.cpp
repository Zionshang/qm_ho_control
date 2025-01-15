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

void HierarchicalWbc::calTau(const Vector4i contact, Vec18 &tau)
{
    // struct timeval t1, t2;
    // gettimeofday(&t1, NULL);
    // gettimeofday(&t2, NULL);
    // std::cout << "Time of setMatrix: \t " << double(t2.tv_usec - t1.tv_usec) / 1000 << "ms" << std::endl;

    initVars(contact);

    Task task0, task1, task2;
    task0 = buildFloatingBaseEomTask(_q, _v) + buildNoContactMotionTask() + buildFrictionConeTask();
    task1 = buildBodyLinearTask() * _wBlinear + buildBodyAngularTask() * _wBangle + buildSwingLegTask() * _wSw;
    if (_est->getWorkMode() != WorkMode::ARM_JOINT)
    {
        task1 = task1 + buildGripperAngularTask() * _wElinear + buildGripperLinearTask() * _wEangle;
    }
    else
    {
        task1 = task1 + buildArmJointTrackingTask() * _wArmJ;
    }

    HoQp hoQp(task1, std::make_shared<HoQp>(task0));
    _solFinal = hoQp.getSolutions();

    tau = _M.bottomRows(_nv - 6) * _solFinal.head(_nv) + _C.bottomRows(_nv - 6) - (_Jst.rightCols(_nv - 6)).transpose() * _solFinal.tail(3 * _nSt);

    // std::cout << "leg tau: \n"
    //           << vec12ToVec34(tau.head(12)) << std::endl;
    // std::cout << "arm tau: \n"
    //           << tau.tail(6).transpose() << std::endl;
}

void HierarchicalWbc::initVars(const Vector4i &contact)
{
    _R << _est->getRotB();
    _q << _est->getPosB(), _est->getQuatB(), vec34ToVec12(_est->getQLeg()), _est->getQArm();                                       // expressed in GLOBAL frame
    _v << _R.transpose() * _est->getVelB(), _R.transpose() * _est->getAngVelB(), vec34ToVec12(_est->getDqLeg()), _est->getDqArm(); // expressed in BODY frame

    // dynamic matrix
    _wbDyn->updateKinematics(_q, _v);

    // feet Jacobian
    _nSt = contact.sum();
    _nSw = 4 - _nSt;
    _idSw.resize(_nSw);

    int indexSt = 0;
    _Jst = MatX(3 * _nSt, ROBOTNV);
    _dJdqst = VecX(3 * _nSt);

    int indexSw = 0;
    _Jsw = MatX(3 * _nSw, ROBOTNV);
    _dJdqsw = VecX(3 * _nSw);

    for (int i = 0; i < 4; i++)
    {
        _wbDyn->setFootJacob(_q, i, _Jfeet[i]);
        _wbDyn->setFootdJdq(_q, _v, i, _dJdqfeet[i]);

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
    _wbDyn->setBodyJacob(_q, _Jbody);
    _wbDyn->setBodydJdq(_q, _v, _dJdqbody);

    // CoM Jacobian
    _wbDyn->setCoMJacob(_q, _Jcom);
    _wbDyn->setCoMdJdq(_q, _v, _dJdqcom);

    // gripper Jacobian
    _wbDyn->setGripperJacob(_q, _Jgrip);
    _wbDyn->setGripperdJdq(_q, _v, _dJdqgrip);

    _dimDecisionVars = _nv + 3 * _nSt;
}

Task HierarchicalWbc::buildFloatingBaseEomTask(const VectorXd &q_gen, const VectorXd &v_gen)
{
    _wbDyn->setMandC(q_gen, v_gen, _M, _C);
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

Task HierarchicalWbc::buildBodyLinearTask()
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    // body linear motion tracking
    // A.leftCols(_nv) = _Jbody.topRows(3);
    // b = _KpPos * (_highCmd->posB - _est->getPosB()) + _KdPos * (_highCmd->velB - _est->getVelB()) - _dJdqbody.head(3);

    // // CoM linear motion tracking
    A.leftCols(_nv) = _Jcom.topRows(3);
    b = _KpPos * (_highCmd->posCoM - _est->getPosCoM()) + _KdPos * (_highCmd->velCoM - _est->getVelCoM()) - _dJdqcom.head(3);
    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildBodyAccTask()
{
    // MatX A = MatX::Zero(6, _dimDecisionVars);
    // A.block(0, 0, 6, 6) = MatX::Identity(6, 6);
}

Task HierarchicalWbc::buildBodyAngularTask()
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    A.leftCols(_nv) = _Jbody.bottomRows(3);
    b = _KpAng * quatErr(_highCmd->quatB, _est->getQuatB()) + _KdAng * (_highCmd->angVelB - _est->getAngVelB()) - _dJdqbody.tail(3);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildSwingLegTask()
{
    MatX A = MatX::Zero(3 * _nSw, _dimDecisionVars);
    VecX b = VecX(3 * _nSw);

    A.leftCols(_nv) = _Jsw;
    for (int i = 0; i < _nSw; i++)
        b.segment(3 * i, 3) = _KpSw * (_highCmd->posF.col(_idSw(i)) - _est->getPosF().col(_idSw(i))) + _KdSw * (_highCmd->velF.col(_idSw(i)) - _est->getVelF().col(_idSw(i))) - _dJdqsw.segment(3 * i, 3);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildGripperLinearTask()
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    A.leftCols(_nv) = _Jgrip.topRows(3);
    b = _KpEePos * (_highCmd->posG - _est->getPosG()) + _KdEePos * (_highCmd->velG - _est->getVelG()) - _dJdqgrip.head(3);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildGripperAngularTask()
{
    MatX A = MatX::Zero(3, _dimDecisionVars);
    VecX b = VecX(3);

    A.leftCols(_nv) = _wEangle * _Jgrip.bottomRows(3);
    b = _KpEeAng * quatErr(_highCmd->quatG, _est->getQuatG()) + _KdEeAng * (_highCmd->angVelG - _est->getAngVelG()) - _dJdqgrip.tail(3);

    return {A, b, MatX(), VecX()};
}

Task HierarchicalWbc::buildArmJointTrackingTask()
{
    MatX A = MatX::Zero(6, _dimDecisionVars);
    VecX b = VecX(6);

    A.block(0, _nv - 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
    b = _KpArmJ * (_highCmd->qAJ - _est->getQArm()) + _KdArmJ * (_highCmd->dqAJ - _est->getDqArm());

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
