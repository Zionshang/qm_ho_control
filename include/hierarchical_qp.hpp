#pragma once

// Ref: https://github.com/bernhardpg/quadruped_locomotion
//      "Perception-less Terrain Adaptation through Whole Body Control and Hierarchical Optimization"

#include "Task.h"
#include <memory>
#include <iostream>
#include <qpOASES.hpp>

class HoQp
{
    // Construct standard form of QP
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using HoQpPtr = std::shared_ptr<HoQp>;

    explicit HoQp(const Task &task) : HoQp(task, nullptr){};
    HoQp(Task task, HoQpPtr higherProblem);

    MatrixXd getStackedZMatrix() const { return _stackedZ; }
    Task getStackedTasks() const { return _stackedTasks; }
    VectorXd getStackedSlackSol() const { return _stackedSlackSol; }
    VectorXd getSolutions() const { return _xPrev + _stackedZPrev * _decisionVarsSol; }

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

    HoQpPtr _higherProblem;                      // solved problems with higher priority
    Task _task;                                  // current task
    Task _stackedTasksPrev;                      // previous tasks
    Task _stackedTasks;                          // previous tasks + current task
    bool _hasEqConstraints, _hasIneqConstraints; // Whether or not the task have a certain constraint

    int _dimSlackVars;     // dimension of slack variables, = rows of inequal constraints
    int dim_decision_vars_;  // dimension of decision variables, = cols of inequal/equal constraints
    int _dimPrevSlackVars; // dimension of the slack variable stacked by the previous tasks

    MatrixXd _stackedZPrev;        // null space of stacked higher task
    MatrixXd _stackedZ;            // null space of stacked (higher tasks + current task)
    VectorXd _xPrev;               // solution of previous higher tasks
    VectorXd _decisionVarsSol;     // solution of current task in decisiton variables
    VectorXd _slackVarsSol;        // solution of current task in slack variables
    VectorXd _stackedSlackSolPrev; // stacked slack variables soluted by each higher tasks
    VectorXd _stackedSlackSol;     // stacked slack variables soluted by (each higher tasks + current task)

    MatrixXd _zeroNvNx; // zero matrix [_dimSlackVars, dim_decision_vars_].Used many times in build QP matrix
    MatrixXd _eyeNvNv;  // Identity matrix [_dimSlackVars, _dimSlackVars. Used many times in build QP matrix

    /************************QP form***************************
                 min  0.5 * x^T H x + c^T x
                s.t.  D^T x <= f
    **********************************************************/
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _H, _D; // matrix in QP form
    VectorXd _c, _f;
};
