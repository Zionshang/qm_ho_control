#pragma once

#include "common/mathTypes.h"
#include <iostream>
/**
 * @brief: convert rotation matrix to ZYX Euler angle
 * @param {Matrix3d} &R: rotation matrix
 * @return {Vector3d} ZYX euler anlge
 */
inline Vector3d rotMat2RPY(const Matrix3d &R)
{
    Vector3d rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = asin(-R(2, 0));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}

/**
 * @brief: convert vector to skew-symmetric matrix
 * @param {Vector3d} &v: vector
 * @return {Matrix3d} skew-symmetric matrix
 */
inline Matrix3d skew(const Vector3d &v)
{
    Matrix3d m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

/**
 * @brief: Eliminate interference from small offsets
 * @param {template T0} a
 * @param {template T1} limit
 * @return {template T0} if abs(a)< limit, a = 0; else a = a;
 */
template <typename T0, typename T1>
inline T0 killSmallOffset(T0 a, const T1 limit)
{
    if ((a > -limit) && (a < limit))
        a = 0;
    return a;
}

/**
 * @brief: Scale value from [minLim,maxLim] to [min,max]
 * @param {template T0} value
 * @param {template T1} min
 * @param {template T1} max
 * @param {double} minLim = -1
 * @param {double} maxLim = -1
 * @return {template T0} value after Scaled
 */
template <typename T0, typename T1>
inline T0 invNormalize(const T0 value, const T1 min, const T1 max, const double minLim = -1, const double maxLim = 1)
{
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

/**
 * @brief set the input value into the limitation [min, max]
 * @param {typename T} input value
 * @param {Vec2} limitation [min, max]
 * @return {typename T} input value which is already saturated
 */
template <typename T>
inline T saturation(const T a, const T &min, const T &max)
{
    if (a < min)
        return min;
    else if (a > max)
        return max;
    else
        return a;
}

/***************************************Vector4d***********************************************/
/**
 * @brief: convert quaternion to rotation matrix
 * @param {Vector4d} &q: quaternion [x,y,z,w]
 * @return {RotMat} rotation matrix
 * @note quaternion must be unit ( ||q|| = 1 )
 */
inline RotMat quat2RotMat(const Vector4d &q)
{
    double e0 = q(0); // x
    double e1 = q(1); // y
    double e2 = q(2); // z
    double e3 = q(3); // w

    RotMat R;
    R << 1 - 2 * (e1 * e1 + e2 * e2), 2 * (e0 * e1 - e3 * e2), 2 * (e0 * e2 + e3 * e1),
        2 * (e0 * e1 + e3 * e2), 1 - 2 * (e0 * e0 + e2 * e2), 2 * (e1 * e2 - e3 * e0),
        2 * (e0 * e2 - e3 * e1), 2 * (e1 * e2 + e3 * e0), 1 - 2 * (e0 * e0 + e1 * e1);
    return R;
}

/**
 * @brief convert quaternion to ZYX Euler angle
 * @param {Vector4d} &q: quaternion [x,y,z,w]
 * @return {Vector3d} ZYX Euler angle
 * @warning quaternion must be unit ( ||q|| = 1 )
 */
inline Vector3d quat2RPY(const Vector4d &q)
{
    double qx = q(0); // x
    double qy = q(1); // y
    double qz = q(2); // z
    double qw = q(3); // w

    Vector3d RPY;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    RPY(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        RPY(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        RPY(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    RPY(2) = std::atan2(siny_cosp, cosy_cosp);

    return RPY;
}

inline Vector4d rpy2Quat(const Vector3d &RPY)
{
    Eigen::AngleAxisd rotationZ(RPY(2), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rotationY(RPY(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotationX(RPY(0), Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rotationZ * rotationY * rotationX;

    return q.coeffs(); // [x, y, z, w]
}
/**
 * @brief: get the quaternion obtained by rotation about the X-axis
 * @param {double} theta: rotation angle
 * @return {Vector4d} quaternion
 */
inline Vector4d quatX(double theta)
{
    return Vector4d(std::sin(theta / 2), 0, 0, std::cos(theta / 2));
}

/**
 * @brief: get the quaternion obtained by rotation about the Y-axis
 * @param {double} theta: rotation angle
 * @return {Vector4d} quaternion
 */
inline Vector4d quatY(double theta)
{
    return Vector4d(0, std::sin(theta / 2), 0, std::cos(theta / 2));
}

/**
 * @brief: get the quaternion obtained by rotation about the Z-axis
 * @param {double} theta: rotation angle
 * @return {Vector4d} quaternion
 */
inline Vector4d quatZ(double theta)
{
    return Vector4d(0, 0, std::sin(theta / 2), std::cos(theta / 2));
}

/**
 * @brief: Multiplication between two quaternion
 * @param {Vector4d} &q1
 * @param {Vector4d} &q2
 * @return {Vector4d} q1 * q2
 */
inline Vector4d quatTimes(const Vector4d &q1, const Vector4d &q2)
{
    Vector4d out = Vector4d(q1(3) * q2(0) + q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1),
                    q1(3) * q2(1) - q1(0) * q2(2) + q1(1) * q2(3) + q1(2) * q2(0),
                    q1(3) * q2(2) + q1(0) * q2(1) - q1(1) * q2(0) + q1(2) * q2(3),
                    q1(3) * q2(3) - q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2));
    return out;
}

/**
 * @brief: Conjugation of quaternions
 * @param {Vector4d} &q
 * @return {Vector4d} q* (Conjugation of quaternions)
 */
inline Vector4d quatConj(const Vector4d &q)
{
    return Vector4d(-q(0), -q(1), -q(2), q(3));
}

/**
 * @brief: caculate the 3D rotation error from quaternion
 * @param {Vector4d} qd: desired quaternion
 * @param {Vector4d} q: current quaternion
 * @return {Vector3d} 3D rotation err. err = qd - q
 */
inline Vector3d quatErr(const Vector4d &qd, const Vector4d &q)
{
    Vector4d qDiff = quatTimes(qd, quatConj(q));
    if (qDiff(3) < 0)
        qDiff = -qDiff;

    return qDiff.head(3);
}

/**
 * @brief: caculate the derivative of quaternion
 * @param {Vector4d} &q: a unit quaternion representing the orientation of B relative to A
 * @param {Vector3d} w: angular velocity, expressed in frame B
 * @return {Vector4d} derivative of quaternion
 */
inline Vector4d quatDeriv(const Vector4d &q, const Vector3d &w)
{
    Eigen::Matrix<double, 4, 3> Q;
    Q << q(3), -q(2), q(1),
        q(2), q(3), -q(0),
        -q(1), q(0), q(3),
        -q(0), -q(1), -q(2);
    return 0.5 * Q * w;
}

/**
 * @brief Cubic polynomial interpolation
 */
inline void cubicSpline(const double &pos0, const double &posf, const double &vel0, const double &velf,
                        const double &t0, const double &tf, const double &tnow,
                        double &posNow, double &velNow)
{
    if (tnow < t0)
    {
        posNow = pos0;
        velNow = 0;
    }
    else if (tnow > tf)
    {
        posNow = posf;
        velNow = 0;
    }
    else
    {
        double a0, a1, a2, a3; // pos = a0 + a1*(tnow-t0) + a2*(tnow-t0)^2 + a3*(tnow-t0)^3
        double h = posf - pos0;
        double T = tf - t0;
        a0 = pos0;
        a1 = vel0;
        a2 = (3 * h - (2 * vel0 + velf) * T) / (T * T);
        a3 = (-2 * h + (vel0 + velf) * T) / (T * T * T);

        posNow = a0 + a1 * (tnow - t0) + a2 * pow(tnow - t0, 2) + a3 * pow(tnow - t0, 3);
        velNow = a1 + 2 * a2 * (tnow - t0) + 3 * a3 * pow(tnow - t0, 2);
    }
}

/**
 * @brief Cubic polynomial interpolation
 */
template <typename T>
inline void cubicSpline(const VectorXd &pos0, const VectorXd &posf, const VectorXd &vel0, const VectorXd &velf,
                        const double &t0, const double &tf, const double &tnow,
                        Eigen::DenseBase<T> &posNow, Eigen::DenseBase<T> &velNow)
{
    if (tnow < t0)
    {
        posNow = pos0;
        velNow.setZero();
    }
    else if (tnow > tf)
    {
        posNow = posf;
        velNow.setZero();
    }
    else
    {
        VectorXd a0, a1, a2, a3; // pos = a0 + a1*(tnow-t0) + a2*(tnow-t0)^2 + a3*(tnow-t0)^3
        VectorXd h = posf - pos0;
        double T0f = tf - t0;
        a0 = pos0;
        a1 = vel0;
        a2 = (3 * h - (2 * vel0 + velf) * T0f) / (T0f * T0f);
        a3 = (-2 * h + (vel0 + velf) * T0f) / (T0f * T0f * T0f);

        posNow = a0 + a1 * (tnow - t0) + a2 * pow(tnow - t0, 2) + a3 * pow(tnow - t0, 3);
        velNow = a1 + 2 * a2 * (tnow - t0) + 3 * a3 * pow(tnow - t0, 2);
    }
}