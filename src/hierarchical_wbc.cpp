#include "hierarchical_wbc.hpp"

HierarchicalWbc::HierarchicalWbc(shared_ptr<PinocchioInterface> pin_interface)
    : pin_interface_(pin_interface)
{
    nv_ = pin_interface->nv();
    fric_coef_ = 0.7;

    // friction cone constraint    fric_mat_ * [Fx Fy Fz]^T <= fric_vec_
    fz_max_ = 2000;
    fz_min_ = 10;
    fric_mat_ << 0, 0, 1,
        0, 0, -1,
        1, 0, -fric_coef_,
        0, 1, -fric_coef_,
        -1, 0, -fric_coef_,
        0, -1, -fric_coef_;
    fric_vec_ << fz_max_, -fz_min_, 0, 0, 0, 0;

    // PD paramerter
    std::string configFile = getProjectPath() + "/config/config.yml";
    paramInit(configFile);

    for (int i = 0; i < 4; i++)
    {
        J_feet_[i].setZero(6, nv_);
    }

    J_body_.setZero(6, nv_);
    J_com_.setZero(3, nv_);
    M_.setZero(nv_, nv_);
    C_.setZero(nv_);
}

void HierarchicalWbc::calTau(const RobotState &robot_state, const RobotState &robot_state_ref,
                             const Vector4i contact, VectorXd &tau)
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

    const auto &pos_body_ref = robot_state_ref.body.pos;
    const auto &vel_body_ref = robot_state_ref.body.vel;
    const auto &quat_body_ref = robot_state_ref.body.quat;
    const auto &angvel_body_ref = robot_state_ref.body.angvel;
    const auto &pos_com_ref = robot_state_ref.pos_com;
    const auto &vel_com_ref = robot_state_ref.vel_com;
    const auto &pos_feet_ref = robot_state_ref.foot.pos;
    const auto &vel_feet_ref = robot_state_ref.foot.vel;
    const auto &pos_arm_ref = robot_state_ref.joint.pos_arm;
    const auto &vel_arm_ref = robot_state_ref.joint.vel_arm;

    dim_decision_vars_ = nv_ + 3 * contact.sum();

    pin_interface_->calcDynamicsMatrix(q_gen, v_gen, M_, C_);
    updateJacobian(q_gen, v_gen, contact);

    Task task0, task1, task2;
    task0 = buildFloatingBaseEomTask();
    task0 = task0 + buildNoContactMotionTask();
    task0 = task0 + buildFrictionConeTask();

    task1 = buildBodyLinearTask(pos_body,vel_body,pos_body_ref,vel_body_ref) * weight_pos_body_;
    // task1 = buildComLinearTask(pos_com, vel_com, pos_com_ref, vel_com_ref) * weight_pos_body_;
    task1 = task1 + buildBodyAngularTask(quat_body, angvel_body, quat_body_ref, angvel_body_ref) * weight_pos_ang_;
    task1 = task1 + buildSwingLegTask(pos_feet, vel_feet, pos_feet_ref, vel_feet_ref) * weight_swing_;
    task1 = task1 + buildArmJointTask(pos_arm, vel_arm, pos_arm_ref, vel_arm_ref) * weight_arm_;

    HoQp hoQp(task1, std::make_shared<HoQp>(task0));
    sol_final_ = hoQp.getSolutions();

    tau = M_.bottomRows(nv_ - 6) * sol_final_.head(nv_) + C_.bottomRows(nv_ - 6) - (J_st_.rightCols(nv_ - 6)).transpose() * sol_final_.tail(3 * num_st_);
}

void HierarchicalWbc::updateJacobian(const VectorXd &pos_gen, const VectorXd &vel_gen, const Vector4i &contact)
{
    // Set up zero-acceleration kinematics, used for calculate dJdq.
    // Because a = J * ddq + dJ * dq. When ddq = 0, a = dJ * dq
    pin_interface_->calcZeroAccKinematics(pos_gen, vel_gen);

    // feet Jacobian
    num_st_ = contact.sum();
    num_sw_ = 4 - num_st_;
    id_sw_.resize(num_sw_);

    int indexSt = 0;
    J_st_.setZero(3 * num_st_, nv_);
    dJdq_st_.setZero(3 * num_st_);

    int indexSw = 0;
    J_sw_.setZero(3 * num_sw_, nv_);
    dJdq_sw_.setZero(3 * num_sw_);

    for (int i = 0; i < 4; i++)
    {
        pin_interface_->calcFootJacobian(pos_gen, i, J_feet_[i]);
        pin_interface_->calcFootJacobianTimesVelocity(pos_gen, vel_gen, i, dJdq_feet_[i]);

        if (contact(i) == 1)
        {
            J_st_.middleRows(3 * indexSt, 3) = J_feet_[i].topRows(3);    // only get the linear part
            dJdq_st_.segment(3 * indexSt, 3) = dJdq_feet_[i].topRows(3); // only get the linear part
            indexSt++;
        }
        else
        {
            J_sw_.middleRows(3 * indexSw, 3) = J_feet_[i].topRows(3);    // only get the linear part
            dJdq_sw_.segment(3 * indexSw, 3) = dJdq_feet_[i].topRows(3); // only get the linear part
            id_sw_(indexSw) = i;
            indexSw++;
        }
    }
    // body Jacobian
    pin_interface_->calcBodyJacobian(pos_gen, J_body_);
    pin_interface_->calcBodyJacobianTimesVelocity(pos_gen, vel_gen, dJdq_body_);

    // CoM Jacobian
    pin_interface_->calcComJacobian(pos_gen, J_com_);
    pin_interface_->calcComJacobianTimesVelocity(pos_gen, vel_gen, dJdq_com_);
}

Task HierarchicalWbc::buildFloatingBaseEomTask()
{
    MatrixXd A = (MatrixXd(6, dim_decision_vars_) << M_.topRows(6), -(J_st_.leftCols(6)).transpose()).finished();
    VectorXd b = -C_.topRows(6);

    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildNoContactMotionTask()
{
    MatrixXd A = MatrixXd::Zero(3 * num_st_, dim_decision_vars_);
    A.topLeftCorner(3 * num_st_, nv_) = J_st_;
    VectorXd b = -dJdq_st_;

    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildFrictionConeTask()
{
    MatrixXd D = MatrixXd::Zero(6 * num_st_, dim_decision_vars_);
    VectorXd f = VectorXd(6 * num_st_);
    for (int i = 0; i < num_st_; i++)
    {
        D.block(6 * i, nv_ + 3 * i, 6, 3) = fric_mat_;
        f.segment(6 * i, 6) = fric_vec_;
    }

    return {MatrixXd(), VectorXd(), D, f};
}

Task HierarchicalWbc::buildBodyLinearTask(const Vector3d &pos_body, const Vector3d &vel_body,
                                          const Vector3d &pos_body_ref, const Vector3d &vel_body_ref)
{
    MatrixXd A = MatrixXd::Zero(3, dim_decision_vars_);
    VectorXd b = VectorXd(3);

    // body linear motion tracking
    A.leftCols(nv_) = J_body_.topRows(3);
    b = kp_pos_ * (pos_body_ref - pos_body) + kd_pos_ * (vel_body_ref - vel_body) - dJdq_body_.head(3);

    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildComLinearTask(const Vector3d &pos_com, const Vector3d &vel_com,
                                         const Vector3d &pos_com_ref, const Vector3d &vel_com_ref)
{
    MatrixXd A = MatrixXd::Zero(3, dim_decision_vars_);
    VectorXd b = VectorXd(3);

    // CoM linear motion tracking
    A.leftCols(nv_) = J_com_.topRows(3);
    b = kp_pos_ * (pos_com_ref - pos_com) + kd_pos_ * (vel_com_ref - vel_com) - dJdq_com_.head(3);
    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildBodyAngularTask(const Quaterniond &quat_body, const Vector3d &angvel_body,
                                           const Quaterniond &quat_body_ref, const Vector3d &angvel_body_ref)
{
    MatrixXd A = MatrixXd::Zero(3, dim_decision_vars_);
    VectorXd b = VectorXd(3);

    A.leftCols(nv_) = J_body_.bottomRows(3);

    // todo: 到底用哪个？
    // Vector3d quat_err = SO3Group().difference(quat_body.coeffs(), quat_body_ref.coeffs());
    Vector3d quat_err = pinocchio::quaternion::log3(quat_body_ref * quat_body.inverse());

    b = kp_ang_ * quat_err + kd_ang_ * (angvel_body_ref - angvel_body) - dJdq_body_.tail(3);
    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildSwingLegTask(const Matrix34d &pos_feet, const Matrix34d &vel_feet,
                                        const Matrix34d &pos_feet_ref, const Matrix34d &vel_feet_ref)
{
    MatrixXd A = MatrixXd::Zero(3 * num_sw_, dim_decision_vars_);
    VectorXd b = VectorXd(3 * num_sw_);

    A.leftCols(nv_) = J_sw_;

    for (int i = 0; i < num_sw_; i++)
    {
        b.segment(3 * i, 3) = kp_sw_ * (pos_feet_ref.col(id_sw_(i)) - pos_feet.col(id_sw_(i))) +
                              kd_sw_ * (vel_feet_ref.col(id_sw_(i)) - vel_feet.col(id_sw_(i))) - dJdq_sw_.segment(3 * i, 3);
    }

    return {A, b, MatrixXd(), VectorXd()};
}

Task HierarchicalWbc::buildArmJointTask(const VectorXd &pos_arm, const VectorXd &vel_arm,
                                        const VectorXd &pos_arm_ref, const VectorXd &vel_arm_ref)
{
    int arm_nq = pos_arm.size();

    MatrixXd A = MatrixXd::Zero(arm_nq, dim_decision_vars_);
    VectorXd b = VectorXd(arm_nq);

    A.block(0, nv_ - arm_nq, arm_nq, arm_nq) = Eigen::MatrixXd::Identity(arm_nq, arm_nq);
    b = kp_arm_ * (pos_arm_ref - pos_arm) + kd_arm_ * (vel_arm_ref - vel_arm);

    return {A, b, MatrixXd(), VectorXd()};
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
    Vector3d kpPos, kdPos, kpAng, kdAng, kpSw, kdSw;
    for (int i = 0; i < 3; i++)
    {
        kpPos(i) = config["kpPos"][i].as<double>();
        kdPos(i) = config["kdPos"][i].as<double>();
        kpAng(i) = config["kpAng"][i].as<double>();
        kdAng(i) = config["kdAng"][i].as<double>();
        kpSw(i) = config["kpSw"][i].as<double>();
        kdSw(i) = config["kdSw"][i].as<double>();
    }

    kp_pos_ = kpPos.asDiagonal();
    kd_pos_ = kdPos.asDiagonal();
    kp_ang_ = kpAng.asDiagonal();
    kd_ang_ = kdAng.asDiagonal();
    kp_sw_ = kpSw.asDiagonal();
    kd_sw_ = kdSw.asDiagonal();

    weight_pos_body_ = config["wBlinear"].as<double>();
    weight_pos_ang_ = config["wBangle"].as<double>();
    weight_swing_ = config["wSw"].as<double>();

    std::cout << "The PD parameter of Body is: \n";
    std::cout << "kpPos:   \t" << kpPos.transpose() << std::endl;
    std::cout << "kdPos:   \t" << kdPos.transpose() << std::endl;
    std::cout << "kpAng:   \t" << kpAng.transpose() << std::endl;
    std::cout << "kdAng:   \t" << kdAng.transpose() << std::endl;
    std::cout << "kpSw:    \t" << kpSw.transpose() << std::endl;
    std::cout << "kdSw:    \t" << kdSw.transpose() << std::endl;
    std::cout << "The weight of Body is: \n";
    std::cout << "weight_pos_body_: " << weight_pos_body_ << "\t\t";
    std::cout << "weight_pos_ang_ : " << weight_pos_ang_ << "\t\t";
    std::cout << "weight_swing_     : " << weight_swing_ << std::endl;

    Vector3d kpEePos, kdEePos, kpEeAng, kdEeAng;
    VectorXd kpArmJ(6), kdArmJ(6); // todo: 不使用魔法数字
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

    // _KpEePos = kpEePos.asDiagonal();
    // _KdEePos = kdEePos.asDiagonal();
    // _KpEeAng = kpEeAng.asDiagonal();
    // _KdEeAng = kdEeAng.asDiagonal();
    kp_arm_ = kpArmJ.asDiagonal();
    kd_arm_ = kdArmJ.asDiagonal();

    // weight_gripper_pos_ = config["wElinear"].as<double>();
    // weight_gripper_ang_ = config["wEangle"].as<double>();
    weight_arm_ = config["wArmJ"].as<double>();

    std::cout << "The PD parameter of Arm is: \n";
    std::cout << "kpEePos: \t" << kpEePos.transpose() << std::endl;
    std::cout << "kdEePos: \t" << kdEePos.transpose() << std::endl;
    std::cout << "kpEeAng: \t" << kpEeAng.transpose() << std::endl;
    std::cout << "kdEeAng: \t" << kdEeAng.transpose() << std::endl;
    std::cout << "kp_arm_: \t" << kpArmJ.transpose() << std::endl;
    std::cout << "kd_arm_: \t" << kdArmJ.transpose() << std::endl;
    std::cout << "The weight of Arm is: \n";
    // std::cout << "wElinear: " << weight_gripper_pos_ << "\t\t";
    // std::cout << "WEangle : " << weight_gripper_ang_ << "\t\t";
    std::cout << "weight_arm_  : " << weight_arm_ << std::endl;
}
