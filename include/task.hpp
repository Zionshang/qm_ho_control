// Ref: https://github.com/bernhardpg/quadruped_locomotion
#pragma once

#include "common/types.hpp"

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
        : A_(std::move(A)), b_(std::move(b)), D_(std::move(D)), f_(std::move(f)) {}
    explicit Task(size_t dim_decision_vars)
        : Task(MatrixXd::Zero(0, dim_decision_vars), VectorXd::Zero(0), MatrixXd::Zero(0, dim_decision_vars), VectorXd::Zero(0)) {};

    const MatrixXd &A() const { return A_; }
    const VectorXd &b() const { return b_; }
    const MatrixXd &D() const { return D_; }
    const VectorXd &f() const { return f_; }

    Task operator+(const Task &rhs) const
    {
        return {concatenateMat(A_, rhs.A_), concatenateVec(b_, rhs.b_), concatenateMat(D_, rhs.D_), concatenateVec(f_, rhs.f_)};
    }

    Task operator*(double rhs) const
    {
        return {
            A_.rows() > 0 ? rhs * A_ : A_,
            b_.rows() > 0 ? rhs * b_ : b_,
            D_.rows() > 0 ? rhs * D_ : D_,
            f_.rows() > 0 ? rhs * f_ : f_};
    }

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

private:
    MatrixXd A_, D_;
    VectorXd b_, f_;
};
