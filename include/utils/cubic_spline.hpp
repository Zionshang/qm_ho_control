#pragma once

class CubicSpline
{
public:
    struct SplineNode
    {
        double time = 0;
        double position = 0;
        double velocity = 0;
    };

    CubicSpline() = default;
    CubicSpline(SplineNode start, SplineNode end);

    double position(double time) const;

    double velocity(double time) const;

    double acceleration(double time) const;

private:
    double normalizedTime(double t) const;

    double t0_;
    double t1_;
    double dt_;

    double c0_;
    double c1_;
    double c2_;
    double c3_;
};
