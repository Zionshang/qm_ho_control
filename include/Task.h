// Ref: https://github.com/bernhardpg/quadruped_locomotion
#pragma once


#include "common/mathTypes.h"

class Task
{
    /* A task is formulated as
        A * x = b
        D * x < f
    */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Task() = default;
    Task(MatX A, VecX b, MatX D, VecX f)
        : _A(std::move(A)), _b(std::move(b)), _D(std::move(D)), _f(std::move(f)) {}
    explicit Task(size_t dimDecisionVars)
        : Task(MatX::Zero(0, dimDecisionVars), VecX::Zero(0), MatX::Zero(0, dimDecisionVars), VecX::Zero(0)){};

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

    MatX _A, _D;
    VecX _b, _f;

    static MatX concatenateMat(const MatX &m1, const MatX &m2)
    {
        if (m1.cols() <= 0)
            return m2;
        else if (m2.cols() <= 0)
            return m1;

        MatX res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;
        return res;
    }

    static VecX concatenateVec(const VecX &v1, const VecX &v2)
    {
        if (v1.cols() <= 0)
            return v2;
        else if (v2.cols() <= 0)
            return v1;

        VecX res(v1.rows() + v2.rows());
        res << v1, v2;
        return res;
    }
};
