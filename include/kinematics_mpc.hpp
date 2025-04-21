#pragma once
#include "common/types.hpp"
#include "aligator/core/traj-opt-problem.hpp"
#include "aligator/modelling/state-error.hpp"
#include "aligator/modelling/costs/sum-of-costs.hpp"
#include "aligator/modelling/dynamics/ode-abstract.hpp"
#include "aligator/modelling/dynamics/integrator-euler.hpp"
#include "aligator/modelling/dynamics/integrator-semi-euler.hpp"
#include "aligator/modelling/multibody/frame-translation.hpp"
#include "aligator/modelling/multibody/frame-placement.hpp"
#include "aligator/modelling/costs/quad-residual-cost.hpp"
#include "aligator/modelling/costs/quad-state-cost.hpp"
#include "aligator/context.hpp"
#include "aligator/solvers/proxddp/solver-proxddp.hpp"
#include <aligator/modelling/multibody/frame-translation.hpp>
#include <proxsuite-nlp/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>
#include <aligator/modelling/multibody/frame-collision.hpp>
#include "aligator/solvers/proxddp/solver-proxddp.hpp"

#include <proxsuite-nlp/modelling/spaces/multibody.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace aligator;
using namespace pinocchio;
using QuadraticResidualCost = QuadraticResidualCostTpl<double>;
using QuadraticControlCost = QuadraticControlCostTpl<double>;
using QuadraticStateCost = QuadraticStateCostTpl<double>;
using StageModel = aligator::context::StageModel;
using TrajOptProblem = aligator::context::TrajOptProblem;
using MultibodyPhaseSpace = proxsuite::nlp::MultibodyPhaseSpace<double>;
using IntegratorSemiImplEuler = dynamics::IntegratorSemiImplEulerTpl<double>;
using IntegratorEuler = dynamics::IntegratorEulerTpl<double>;
using FramePlacementResidual = FramePlacementResidualTpl<double>;
using StateErrorResidual = StateErrorResidualTpl<double>;
using FrameVelocityResidual = FrameVelocityResidualTpl<double>;
using FunctionSliceXpr = FunctionSliceXprTpl<double>;
using PolymorphicConstraintSet = xyz::polymorphic<aligator::ConstraintSetTpl<double>>;
using BoxConstraint = proxsuite::nlp::BoxConstraintTpl<double>;
using CostStack = CostStackTpl<double>;
using FrameTranslationResidual = aligator::FrameTranslationResidualTpl<double>;
using FrameCollisionResidual = FrameCollisionResidualTpl<double>;
using SolverProxDDP = SolverProxDDPTpl<double>;

class KinematicsMPC
{
public:
    KinematicsMPC(int nsteps = 40, double dt = 0.05);

    void setTarget(const VectorXd &x, const Vector3d &position, const Quaterniond &orientation);
    void solve(const VectorXd &x0);
    const std::vector<VectorXd> &getSolution() const { return x_sol_; };

    const pinocchio::Model &model() { return model_; }
    int nq() { return nq_; }
    int nv() { return nv_; }
    double dt() { return dt_; }

private:
    void setStageEeTarget(const std::size_t t, const pinocchio::SE3 &ee_target);   // 更新阶段末端执行器目标
    void setStateTarget(const std::size_t t, const Eigen::VectorXd &state_target); // 更新阶段状态目标
    void setTerminalEeTarget(const pinocchio::SE3 &term_target);                   // 更新终端末端执行器目标

    CostStack *getCostStack(std::size_t t); // 获取阶段的成本栈
    CostStack *getTerminalCostStack();      // 获取终端成本栈
    void setupProblem(int nsteps);
    CostStack createTerminalCost(const MultibodyPhaseSpace &space, int nu, const pinocchio::SE3 &target_pose,
                                 const FrameIndex frame_id, const pinocchio::Model &model, const Matrix6d &W_vel, const Matrix6d &W_ee);

    void addJointPositionLimits(StageModel &stage, const MultibodyPhaseSpace &space, int nu,
                                const Model &model);
    void addCollisionConstraints(StageModel &stage,
                                 const MultibodyPhaseSpace &space,
                                 const pinocchio::Model &model,
                                 const pinocchio::GeometryModel &geom_model,
                                 int nu,
                                 double min_distance);
    void addBaseVelocityLimits(StageModel &stage, const MultibodyPhaseSpace &space, int nu,
                               const Model &model, double max_vel);

    void addBaseHeightConstraint(StageModel &stage,
                                 const MultibodyPhaseSpace &space,
                                 const pinocchio::Model &model,
                                 int nu,
                                 double min_height,
                                 double max_height);
    pinocchio::Model model_;
    pinocchio::GeometryModel geom_model_;
    pinocchio::Data data_;

    std::string frame_name_;
    pinocchio::SE3 target_pose_;
    std::unique_ptr<TrajOptProblem> problem_;
    std::unique_ptr<SolverProxDDP> solver_;
    std::vector<VectorXd> x_sol_;
    std::vector<VectorXd> u_sol_;

    int nq_;
    int nv_;
    double dt_;
    bool target_set_;
    Matrix6d W_ee_;  // 末端执行器权重
    Matrix6d W_vel_; // 速度权重
    MatrixXd W_u_;   // 控制量权重
    VectorXd W_x_diag_;   // 状态权重
};