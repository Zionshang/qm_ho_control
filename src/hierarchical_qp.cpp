#include "hierarchical_qp.hpp"

HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : current_task_(std::move(task)),
                                                     higher_problem_(std::move(higherProblem))
{
    initVars();
    formulateProblem();
    solveProblem();
    // prepare for next problem
    buildZMatrix();
    stackSlackSolutions();
    final_sol_ = x_prev_ + stacked_Z_prev_ * decision_vars_sol_;
}

void HoQp::initVars()
{
    // Task variables
    dim_slack_vars_ = current_task_.f().rows();
    has_eq_constraints_ = current_task_.b().rows() > 0;
    has_ineq_constraints_ = dim_slack_vars_ > 0;

    // Pre-Task variables
    if (higher_problem_ == nullptr) // The task with the highest priority should be solved first
    {

        dim_decision_vars_ = current_task_.A().cols();
        stacked_tasks_prev_ = Task(dim_decision_vars_);
        stacked_Z_prev_ = MatrixXd::Identity(dim_decision_vars_, dim_decision_vars_);
        stacked_slack_sol_prev_ = VectorXd::Zero(0);
        x_prev_ = VectorXd::Zero(dim_decision_vars_);
        dim_prev_slack_vars_ = 0;
    }
    else
    {
        stacked_Z_prev_ = higher_problem_->getStackedZMatrix();
        stacked_tasks_prev_ = higher_problem_->getStackedTasks();
        stacked_slack_sol_prev_ = higher_problem_->getStackedSlackSol();
        x_prev_ = higher_problem_->getSolutions();
        dim_prev_slack_vars_ = higher_problem_->getStackedTasks().D().rows();
        dim_decision_vars_ = stacked_Z_prev_.cols();
    }

    stacked_tasks_ = current_task_ + stacked_tasks_prev_;

    // Init convenience matrices
    eye_Nv_Nv = MatrixXd::Identity(dim_slack_vars_, dim_slack_vars_);
    zero_Nv_Nx_ = MatrixXd::Zero(dim_slack_vars_, dim_decision_vars_);
}

void HoQp::formulateProblem()
{
    buildHMatrix();
    buildCVector();
    buildDMatrix();
    buildFVector();
}

void HoQp::buildHMatrix()
{
    MatrixXd ZtAtAZ(dim_decision_vars_, dim_decision_vars_); // t means transpose
    if (has_eq_constraints_)
    {
        // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
        MatrixXd ACurrZPrev = current_task_.A() * stacked_Z_prev_;
        // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
        ZtAtAZ = ACurrZPrev.transpose() * ACurrZPrev + 1e-12 * MatrixXd::Identity(dim_decision_vars_, dim_decision_vars_);
    }
    else
        ZtAtAZ.setZero();

    H_ = (MatrixXd(dim_decision_vars_ + dim_slack_vars_, dim_decision_vars_ + dim_slack_vars_) // clang-format off
              <<   ZtAtAZ, zero_Nv_Nx_.transpose(), 
                zero_Nv_Nx_, eye_Nv_Nv) // clang-format on
             .finished();
}

void HoQp::buildCVector()
{
    VectorXd c = VectorXd::Zero(dim_decision_vars_ + dim_slack_vars_);
    VectorXd zeroVec = VectorXd::Zero(dim_slack_vars_);
    VectorXd temp(dim_decision_vars_);
    if (has_eq_constraints_)
        temp = (current_task_.A() * stacked_Z_prev_).transpose() * (current_task_.A() * x_prev_ - current_task_.b());
    else
        temp.setZero();

    c_ = (VectorXd(dim_decision_vars_ + dim_slack_vars_) // clang-format off
             << temp, 
                zeroVec) // clang-format on
             .finished();
}

void HoQp::buildDMatrix()
{
    MatrixXd stackedZero = MatrixXd::Zero(dim_prev_slack_vars_, dim_slack_vars_);
    MatrixXd DCurrZ;
    if (has_ineq_constraints_)
        DCurrZ = current_task_.D() * stacked_Z_prev_;
    else
        DCurrZ = MatrixXd::Zero(0, dim_decision_vars_);

    // NOTE: This is upside down compared to the paper, but more consistent with the rest of the algorithm
    D_ = (MatrixXd(2 * dim_slack_vars_ + dim_prev_slack_vars_, dim_decision_vars_ + dim_slack_vars_) // clang-format off
              << zero_Nv_Nx_                           , -eye_Nv_Nv,
                 stacked_tasks_prev_.D() * stacked_Z_prev_, stackedZero,
                 DCurrZ                              , -eye_Nv_Nv) // clang-format on
             .finished();
}

void HoQp::buildFVector()
{
    VectorXd zeroVec = VectorXd::Zero(dim_slack_vars_);
    VectorXd fMinusDXPrev;
    if (has_ineq_constraints_)
        fMinusDXPrev = current_task_.f() - current_task_.D() * x_prev_;
    else
        fMinusDXPrev = VectorXd::Zero(0);

    f_ = (VectorXd(2 * dim_slack_vars_ + dim_prev_slack_vars_) // clang-format off
              << zeroVec,
                stacked_tasks_prev_.f() - stacked_tasks_prev_.D() * x_prev_ + stacked_slack_sol_prev_,
                fMinusDXPrev)
             .finished(); // clang-format on
}

void HoQp::buildZMatrix()
{
    if (has_eq_constraints_)
        stacked_Z_ = stacked_Z_prev_ * (current_task_.A() * stacked_Z_prev_).fullPivLu().kernel();
    else
        stacked_Z_ = stacked_Z_prev_;
}

void HoQp::solveProblem()
{
    auto qpProblem = qpOASES::QProblem(dim_decision_vars_ + dim_slack_vars_, f_.size());
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(H_.data(), c_.data(), D_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
    VectorXd qpSol(dim_decision_vars_ + dim_slack_vars_);

    qpProblem.getPrimalSolution(qpSol.data());

    decision_vars_sol_ = qpSol.head(dim_decision_vars_);
    slack_vars_sol_ = qpSol.tail(dim_slack_vars_);
}

void HoQp::stackSlackSolutions()
{
    if (higher_problem_ != nullptr)
        stacked_slack_sol_ = Task::concatenateVec(higher_problem_->getStackedSlackSol(), slack_vars_sol_);
    else
        stacked_slack_sol_ = slack_vars_sol_;
}