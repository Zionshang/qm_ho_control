#pragma once

// Ref: https://github.com/bernhardpg/quadruped_locomotion
//      "Perception-less Terrain Adaptation through Whole Body Control and Hierarchical Optimization"

#include "task.hpp"
#include <memory>
#include <iostream>
#include <qpOASES.hpp>

class HoQp
{
    // Construct standard form of QP
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using HoQpPtr = std::shared_ptr<HoQp>;

    explicit HoQp(const Task &task) : HoQp(task, nullptr) {};
    HoQp(Task task, HoQpPtr higherProblem);

    const MatrixXd &getStackedZMatrix() const { return stacked_Z_; }
    const Task &getStackedTasks() const { return stacked_tasks_; }
    const VectorXd &getStackedSlackSol() const { return stacked_slack_sol_; }
    const VectorXd &getSolutions() const { return final_sol_; }

private:
    void initVars();
    void formulateProblem();
    void buildHMatrix();
    void buildCVector();
    void buildDMatrix();
    void buildFVector();
    void buildZMatrix();
    void solveProblem();
    void stackSlackSolutions();

    HoQpPtr higher_problem_;                         // solved problems with higher priority
    Task current_task_;                              // current task
    Task stacked_tasks_prev_;                        // previous tasks
    Task stacked_tasks_;                             // previous tasks + current task
    bool has_eq_constraints_, has_ineq_constraints_; // Whether or not the task have a certain constraint

    int dim_slack_vars_;      // dimension of slack variables, = rows of inequal constraints
    int dim_decision_vars_;   // dimension of decision variables, = cols of inequal/equal constraints
    int dim_prev_slack_vars_; // dimension of the slack variable stacked by the previous tasks

    MatrixXd stacked_Z_prev_;         // null space of stacked higher task
    MatrixXd stacked_Z_;              // null space of stacked (higher tasks + current task)
    VectorXd x_prev_;                 // solution of previous higher tasks
    VectorXd decision_vars_sol_;      // solution of current task in decisiton variables
    VectorXd slack_vars_sol_;         // solution of current task in slack variables
    VectorXd stacked_slack_sol_prev_; // stacked slack variables soluted by each higher tasks
    VectorXd stacked_slack_sol_;      // stacked slack variables soluted by (each higher tasks + current task)
    VectorXd final_sol_;              // final solution of all tasks

    MatrixXd zero_Nv_Nx_; // zero matrix [dim_slack_vars_, dim_decision_vars_].Used many times in build QP matrix
    MatrixXd eye_Nv_Nv;   // Identity matrix [dim_slack_vars_, dim_slack_vars_. Used many times in build QP matrix

    /************************QP form***************************
                 min  0.5 * x^T H x + c^T x
                s.t.  D^T x <= f
    **********************************************************/
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_, D_; // matrix in QP form
    VectorXd c_, f_;
};
