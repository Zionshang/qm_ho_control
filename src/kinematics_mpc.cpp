#include "kinematics_mpc.hpp"
#include <pinocchio/parsers/srdf.hpp>
#include "yaml-cpp/yaml.h"
#include "utils/yaml_loader.hpp"
struct QuadrupedArmDynamics : dynamics::ODEAbstractTpl<double>
{
    using Base = dynamics::ODEAbstractTpl<double>;
    using ODEData = dynamics::ContinuousDynamicsDataTpl<double>;

    QuadrupedArmDynamics(const pinocchio::Model &model, int nu)
        : Base(MultibodyPhaseSpace(model), nu)
    {
        nq = model.nq;
        nv = model.nv;
    }

    void forward(const ConstVectorRef &x, const ConstVectorRef &u,
                 ODEData &data) const override
    {
        data.xdot_.head(nv) = x.tail(nv); // 速度
        data.xdot_.tail(nv) = u;          // 加速度
    }

    void dForward(const ConstVectorRef &x, const ConstVectorRef &u,
                  ODEData &data) const override
    {
        data.Jx_.setZero();
        data.Jx_.topRightCorner(nv, nv).setIdentity();
        data.Ju_.setZero();
        data.Ju_.bottomRows(nv).setIdentity();
    }

    int nq;
    int nv;
};

KinematicsMPC::KinematicsMPC(int nsteps, double dt) : dt_(dt)
{
    std::string urdf_path = "/home/jyx/cpp_workspace/qm_ho_control/urdf/bqr3_arm_noleg.urdf";
    std::string frame_name = "wrist_yall_joint";
    std::string srdf_path = "/home/jyx/cpp_workspace/qm_ho_control/urdf/bqr3_arm_noleg.srdf";
    std::string yaml_filepath = "/home/jyx/cpp_workspace/qm_ho_control/config/config.yml";
    pinocchio::urdf::buildModel(urdf_path, JointModelFreeFlyer(), model_);
    pinocchio::srdf::loadReferenceConfigurations(model_, srdf_path, false);
    pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_);
    geom_model_.addAllCollisionPairs();
    data_ = pinocchio::Data(model_);

    YamlParams yaml_params(yaml_filepath);
    W_ee_ = yaml_params.W_ee_diag.asDiagonal().toDenseMatrix();
    W_vel_ = yaml_params.W_vel_diag.asDiagonal().toDenseMatrix();
    W_u_ = yaml_params.W_u_diag.asDiagonal().toDenseMatrix();
    W_x_diag_ = VectorXd::Zero(yaml_params.w_x_body_pos.size() +
                                      yaml_params.w_x_arm_pos.size() +
                                      yaml_params.w_x_body_vel.size() +
                                      yaml_params.w_x_arm_vel.size());
    W_x_diag_ << yaml_params.w_x_body_pos, yaml_params.w_x_arm_pos,
        yaml_params.w_x_body_vel, yaml_params.w_x_arm_vel;

    nq_ = model_.nq;
    nv_ = model_.nv;

    frame_name_ = frame_name;

    const double TOL = 1e-6;
    const double mu_init = 1e-4; // 1e-2;

    solver_ = std::make_unique<SolverProxDDP>(TOL, mu_init);
    solver_->verbose_ = aligator::VerboseLevel::VERYVERBOSE;
    solver_->sa_strategy_ = StepAcceptanceStrategy::LINESEARCH_ARMIJO;
    solver_->linear_solver_choice = LQSolverChoice::PARALLEL;
    solver_->setNumThreads(4);
    solver_->rollout_type_ = RolloutType::LINEAR;

    setupProblem(nsteps);
    solver_->setup(*problem_);
    solver_->run(*problem_);

    solver_->max_iters = 100;
    x_sol_ = solver_->results_.xs;
    u_sol_ = solver_->results_.us;
}

void KinematicsMPC::setTarget(const VectorXd &x, const Vector3d &position, const Quaterniond &orientation)
{
    target_pose_ = pinocchio::SE3(orientation, position);

    // 获取初始状态的末端执行器位姿
    pinocchio::forwardKinematics(model_, data_, x.head(model_.nq));        // 计算正向运动学
    pinocchio::updateFramePlacements(model_, data_);                       // 更新所有帧的位姿
    pinocchio::SE3 start_pose = data_.oMf[model_.getFrameId(frame_name_)]; // 获取末端执行器的初始位姿

    // 计算插值并设置目标
    size_t nsteps = problem_->numSteps();
    for (size_t t = 0; t < nsteps; ++t)
    {
        double alpha = static_cast<double>(t) / (nsteps - 1);
        pinocchio::SE3 interpolated_pose = pinocchio::SE3::Interpolate(start_pose, target_pose_, alpha);

        setStageEeTarget(t, interpolated_pose);
        setStateTarget(t, x);
    }

    // 设置终端阶段的末端执行器目标
    setTerminalEeTarget(target_pose_);
    std::cout << "target_pose_: " << target_pose_.translation().transpose() << std::endl;
    std::cout << "target_pose_: " << Quaterniond(target_pose_.rotation()).coeffs().transpose() << std::endl;
}

CostStack KinematicsMPC::createTerminalCost(const MultibodyPhaseSpace &space, int nu, const pinocchio::SE3 &target_pose,
                                            const FrameIndex frame_id, const pinocchio::Model &model, const Matrix6d &W_vel, const Matrix6d &W_ee)
{
    auto terminal_cost = CostStack(space, nu);
    auto pose_residual = FramePlacementResidual(space.ndx(), nu, model, target_pose, frame_id);
    auto velocity_residual = FrameVelocityResidual(
        space.ndx(), nu, model, pinocchio::Motion::Zero(), frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    terminal_cost.addCost("terminal_pose_cost", QuadraticResidualCost(space, pose_residual, W_ee));
    terminal_cost.addCost("terminal_velocity_cost", QuadraticResidualCost(space, velocity_residual, W_vel));

    return terminal_cost;
}
void KinematicsMPC::setupProblem(int nsteps)
{
    const int nq = model_.nq;
    const int nv = model_.nv;
    const int nu = nv;

    auto space = MultibodyPhaseSpace(model_);
    const FrameIndex frame_id = model_.getFrameId(frame_name_);

    // 初始状态
    std::string reference_configuration_name = "standing";
    VectorXd x0(nq + nv);
    x0.head(nq) = model_.referenceConfigurations[reference_configuration_name]; // 设置前 nq 部分
    std::cout << "x0.head(nq): " << x0.head(nq).transpose() << std::endl;
    x0.tail(nv).setZero();
    // 提取目标位置
    pinocchio::forwardKinematics(model_, data_, x0.head(nq)); // 计算正向运动学
    pinocchio::updateFramePlacements(model_, data_);          // 更新所有帧的位姿
    // 获取末端执行器的初始位置
    pinocchio::SE3 start_pose = data_.oMf[frame_id]; // 末端执行器的初始位姿
    target_pose_ = start_pose;

    // // 设置权重矩阵
    // Vector6d W_ee_diag = VectorXd::Zero(6);
    // W_ee_diag << 10000.0, 10000.0, 10000.0, 5000.0, 5000.0, 5000.0; // 前三个位置后三个方向
    // Matrix6d W_ee = W_ee_diag.asDiagonal();
    // Vector6d W_vel_diag = VectorXd::Zero(6);
    // W_vel_diag << 1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0; // 前三个平移后三个旋转
    // Matrix6d W_vel = W_vel_diag.asDiagonal();
    // VectorXd W_u_diag(nu);
    // W_u_diag << 1, 1, 100, 100, 100, 10,
    //     1e-2, 1e-2, 1e-2, 1e-2, 1e-2;     // 五自由度机械臂相关调整
    // MatrixXd W_u = W_u_diag.asDiagonal(); // 控制量权重
    // VectorXd W_x_diag = VectorXd::Zero(space.ndx());
    // Vector6d w_x_body_pos = VectorXd::Constant(6, 100.0);
    // Vector5d w_x_arm_pos = VectorXd::Constant(5, 1.0);
    // Vector6d w_x_body_vel = VectorXd::Constant(6, 10.0);
    // Vector5d w_x_arm_vel = VectorXd::Constant(5, 1.0);
    // W_x_diag << w_x_body_pos, w_x_arm_pos, w_x_body_vel, w_x_arm_vel;

    // 创建动力学模型
    auto ode = QuadrupedArmDynamics(model_, nu);
    auto discrete_dyn = IntegratorEuler(ode, dt_);

    // 创建阶段向量
    std::vector<xyz::polymorphic<aligator::context::StageModel>> stages;

    // 为每个阶段动态添加与插值点相关的成本
    for (size_t i = 0; i < nsteps; ++i)
    {
        // 动态计算当前插值点
        double alpha = static_cast<double>(i) / (nsteps - 1); // 插值比例
        pinocchio::SE3 current_target_pose = pinocchio::SE3::Interpolate(start_pose, target_pose_, alpha);

        // 创建成本
        auto rcost = CostStack(space, nu);
        auto ee_cost = QuadraticResidualCost(space, FramePlacementResidual(space.ndx(), nu, model_, current_target_pose, frame_id), 0.1 * W_ee_);
        auto qcc = QuadraticControlCost(space, nu, W_u_);
        auto state_cost = QuadraticStateCost(space, nu, x0, W_x_diag_.asDiagonal());
        rcost.addCost("control_cost", qcc);
        rcost.addCost("state_cost", state_cost);
        rcost.addCost("ee_cost", ee_cost);

        // 创建阶段模型
        auto stage = StageModel(rcost, discrete_dyn);
        addJointPositionLimits(stage, space, nu, model_);
        addBaseHeightConstraint(stage, space, model_, nu, 0.40, 0.60);
        addCollisionConstraints(stage, space, model_, geom_model_, nu, 0.05);
        addBaseVelocityLimits(stage, space, nu, model_, 2.0);

        // 添加阶段到阶段向量
        stages.push_back(stage);
    }

    // 添加到终端成本
    auto term_cost = createTerminalCost(space, nu, target_pose_, frame_id, model_, W_vel_, W_ee_);

    problem_ = std::make_unique<TrajOptProblem>(x0, std::move(stages), term_cost);
}

void KinematicsMPC::solve(const VectorXd &x0)
{
    // Recede all horizons
    x_sol_.erase(x_sol_.begin());
    x_sol_[0] = x0;
    x_sol_.push_back(x_sol_.back());

    u_sol_.erase(u_sol_.begin());
    u_sol_.push_back(u_sol_.back());

    problem_->setInitState(x0);

    auto start_time = std::chrono::high_resolution_clock::now();
    solver_->run(*problem_, x_sol_, u_sol_);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    fmt::print("solver.run execution time: {} ms\n", duration);

    x_sol_ = solver_->results_.xs;
    u_sol_ = solver_->results_.us;
}

void KinematicsMPC::setStageEeTarget(const std::size_t t, const pinocchio::SE3 &ee_target)
{
    CostStack *cs = getCostStack(t);
    QuadraticResidualCost *qrc = cs->getComponent<QuadraticResidualCost>("ee_cost");
    FramePlacementResidual *cfr = qrc->getResidual<FramePlacementResidual>();
    cfr->setReference(ee_target);
}

void KinematicsMPC::setStateTarget(const std::size_t t, const Eigen::VectorXd &state_target)
{
    CostStack *cs = getCostStack(t);
    QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("state_cost");

    qsc->setTarget(state_target);
}

void KinematicsMPC::setTerminalEeTarget(const pinocchio::SE3 &ee_target)
{
    CostStack *cs = getTerminalCostStack();
    QuadraticResidualCost *qrc = cs->getComponent<QuadraticResidualCost>("terminal_pose_cost");
    FramePlacementResidual *cfr = qrc->getResidual<FramePlacementResidual>();
    cfr->setReference(ee_target);
}
CostStack *KinematicsMPC::getCostStack(std::size_t t)
{
    if (t >= problem_->numSteps())
    {
        throw std::runtime_error("Stage index exceeds stage vector size");
    }
    CostStack *cs = dynamic_cast<CostStack *>(&*problem_->stages_[t]->cost_);

    return cs;
}
CostStack *KinematicsMPC::getTerminalCostStack()
{
    CostStack *cs = dynamic_cast<CostStack *>(&*problem_->term_cost_);

    return cs;
}
void KinematicsMPC::addJointPositionLimits(StageModel &stage, const MultibodyPhaseSpace &space, int nu,
                                           const Model &model)
{
    auto state_error = StateErrorResidual(space, nu, space.neutral());

    int start_idx = 7;                  // 浮动基座后的关节起始索引
    int njoints = model.nq - start_idx; // 关节数量

    // 3. 创建索引向量，选择关节位置
    std::vector<int> indices = {7, 8, 9, 10, 11}; // //五自由度机械臂相关调整

    // 4. 使用切片表达式获取关节位置函数
    auto pos_func = FunctionSliceXpr(state_error, indices);

    // 获取关节位置限制
    VectorXd pos_min = model.lowerPositionLimit.segment(start_idx, njoints).array() + 0.05;
    VectorXd pos_max = model.upperPositionLimit.segment(start_idx, njoints).array() - 0.05;
    // std ::cout << "关节位置限制: " << pos_min.transpose() << " " << pos_max.transpose() << std::endl;

    stage.addConstraint(pos_func, BoxConstraint(-pos_max, -pos_min));
}

void KinematicsMPC::addCollisionConstraints(StageModel &stage,
                                            const MultibodyPhaseSpace &space,
                                            const pinocchio::Model &model,
                                            const pinocchio::GeometryModel &geom_model,
                                            int nu,
                                            double min_distance)
{
    std::vector<std::size_t> collision_ids = {1, 2}; // 使用ID 1和2的碰撞对
    for (const auto &index : collision_ids)
    {
        auto collision_residual = FrameCollisionResidual(
            space.ndx(), nu, model, geom_model, index); // 使用碰撞对索引创建残差

        // 设置安全距离约束
        VectorXd min_dist(1), max_dist(1);
        min_dist << min_distance; // 最小安全距离
        max_dist << 1000;

        auto collision_constraint = BoxConstraint(min_dist, max_dist);

        // // 打印信息
        // std::cout << "添加碰撞约束: " << geom_model.geometryObjects[geom_model.collisionPairs[index].first].name
        //           << " - " << geom_model.geometryObjects[geom_model.collisionPairs[index].second].name
        //           << "，安全距离: " << min_distance << " 米" << std::endl;

        // 添加约束到阶段模型
        stage.addConstraint(collision_residual, collision_constraint);
    }
}

void KinematicsMPC::addBaseVelocityLimits(StageModel &stage, const MultibodyPhaseSpace &space, int nu,
                                          const Model &model, double max_vel)
{
    // 获取机身（base_link）对应的框架ID
    FrameIndex base_frame_id = model.getFrameId("base_link");

    // 创建零参考速度（我们希望限制速度，所以参考值为零）
    pinocchio::Motion zero_vel = pinocchio::Motion::Zero();

    // 使用FrameVelocityResidual获取机身速度
    auto base_vel_residual = FrameVelocityResidual(
        space.ndx(),                                   // 状态空间维度
        nu,                                            // 控制输入维度
        model,                                         // Pinocchio模型
        zero_vel,                                      // 参考速度（零速度）
        base_frame_id,                                 // 框架ID
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED // 参考坐标系类型
    );

    // 只选择平移速度（前3个元素）
    std::vector<int> indices = {0, 1, 2}; // X, Y, Z 平移速度
    auto vel_func = FunctionSliceXpr(base_vel_residual, indices);

    // 设置速度限制
    VectorXd vel_min = VectorXd::Constant(3, -max_vel);
    VectorXd vel_max = VectorXd::Constant(3, max_vel);

    // 创建约束
    auto box_constraint = PolymorphicConstraintSet(BoxConstraint(vel_min, vel_max));

    // 添加约束到模型
    stage.addConstraint(vel_func, box_constraint);
}

void KinematicsMPC::addBaseHeightConstraint(StageModel &stage,
                                            const MultibodyPhaseSpace &space,
                                            const pinocchio::Model &model,
                                            int nu,
                                            double min_height,
                                            double max_height)
{
    // 获取机身（base_link）对应的框架ID
    const FrameIndex body_frame_id = model.getFrameId("base_link");

    // 创建机身的平移残差（FrameTranslationResidual）
    auto body_frame_error = FrameTranslationResidual(space.ndx(), nu, model, Vector3d::Zero(), body_frame_id);

    // 提取 z 坐标（高度）
    std::vector<int> body_slice_id = {2}; // z 坐标索引
    FunctionSliceXpr body_slice = FunctionSliceXpr(body_frame_error, body_slice_id);

    // 设置高度范围
    VectorXd min(body_slice_id.size()), max(body_slice_id.size());
    min << min_height; // 最小高度
    max << max_height; // 最大高度

    // 添加高度约束到阶段模型
    stage.addConstraint(body_slice, BoxConstraint(min, max));
}
