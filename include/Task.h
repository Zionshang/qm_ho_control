// Ref: https://github.com/bernhardpg/quadruped_locomotion
#pragma once


#include "common/math_types.hpp"

class Task
{
    /* A task is formulated as
        A * x = b
        D * x < f
    */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Task() = default;
    Task(MatrixXd A, VectorXd b, MatrixXd D, VectorXd f)
        : _A(std::move(A)), _b(std::move(b)), _D(std::move(D)), _f(std::move(f)) {}
    explicit Task(size_t dimDecisionVars)
        : Task(MatrixXd::Zero(0, dimDecisionVars), VectorXd::Zero(0), MatrixXd::Zero(0, dimDecisionVars), VectorXd::Zero(0)){};

    Task operator+(const Task &rhs) const
    {
        return {concatenateMat(_A, rhs._A), concatenateVec(_b, rhs._b), concatenateMat(_D, rhs._D), concatenateVec(_f, rhs._f)};
    }

    Task operator*(double rhs) const
    {
        return {
            _A.rows() > 0 ? rhs * _A : _A,
            _b.rows() > 0 ? rhs * _b : _b,
            _D.rows() > 0 ? rhs * _D : _D,
            _f.rows() > 0 ? rhs * _f : _f};
    }

    MatrixXd _A, _D;
    VectorXd _b, _f;

    static MatrixXd concatenateMat(const MatrixXd &m1, const MatrixXd &m2)
    {
        if (m1.cols() <= 0)
            return m2;
        else if (m2.cols() <= 0)
            return m1;

        MatrixXd res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;
        return res;
    }

    static VectorXd concatenateVec(const VectorXd &v1, const VectorXd &v2)
    {
        if (v1.cols() <= 0)
            return v2;
        else if (v2.cols() <= 0)
            return v1;

        VectorXd res(v1.rows() + v2.rows());
        res << v1, v2;
        return res;
    }
};
