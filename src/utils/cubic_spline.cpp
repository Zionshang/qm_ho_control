#include <utils/cubic_spline.hpp>

CubicSpline::CubicSpline(SplineNode start, SplineNode end)
{
    t0_ = start.time;
    t1_ = end.time;
    dt_ = end.time - start.time;

    double dp = end.position - start.position;
    double dv = end.velocity - start.velocity;

    double dc0_ = 0.0;
    double dc1_ = start.velocity;
    double dc2_ = -(3.0 * start.velocity + dv);
    double dc3_ = (2.0 * start.velocity + dv);

    c0_ = dc0_ * dt_ + start.position;
    c1_ = dc1_ * dt_;
    c2_ = dc2_ * dt_ + 3.0 * dp;
    c3_ = dc3_ * dt_ - 2.0 * dp;
}

double CubicSpline::position(double time) const
{
    double tn = normalizedTime(time);
    return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

double CubicSpline::velocity(double time) const
{
    double tn = normalizedTime(time);
    return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

double CubicSpline::acceleration(double time) const
{
    double tn = normalizedTime(time);
    return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

double CubicSpline::normalizedTime(double t) const
{
    return (t - t0_) / dt_;
}
