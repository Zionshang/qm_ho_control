#include "model/QuadrupedLeg.h"
#include <iostream>

QuadrupedLeg::QuadrupedLeg(int legID)
{

    _abadLink = 0.0868;
    _upperLink = 0.25;
    _lowerLink = 0.25;

    if (legID == 0 || legID == 1)
        _sideSign = -1;
    else if (legID == 2 || legID == 3)
        _sideSign = 1;
    else
    {
        std::cout << "Leg ID is incorrect!" << std::endl;
        exit(-1);
    }
}

/**
 * @brief Forward kinematics of leg. Given joint angle, calculate the position of foot
 * @param {Vec3} q: joint angle
 * @return {Vec3} position of foot, expressed in ABAD frame
 */
Vec3 QuadrupedLeg::FK(const Vec3 &q)
{
    float l1 = _sideSign * _abadLink;
    float l2 = -_upperLink;
    float l3 = -_lowerLink;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    Vec3 posF;

    posF(0) = l3 * s23 + l2 * s2;
    posF(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    posF(2) = l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return posF;
}

/**
 * @brief Inverse kinematics. Given foot position, calculate the joint angle.
 * @param {Vec3} posF: position of foot, expressed in ABAD frame
 * @return {Vec3} joint angle
 */
Vec3 QuadrupedLeg::IK(const Vec3 &posF)
{
    Vec3 q;

    float b2y = _abadLink * _sideSign;
    float b3z = -_upperLink;
    float b4z = -_lowerLink;
    float a = _abadLink;
    float c = sqrt(pow(posF(0), 2) + pow(posF(1), 2) + pow(posF(2), 2));
    float b = sqrt(pow(c, 2) - pow(a, 2));
    float L = sqrt(pow(posF(1), 2) + pow(posF(2), 2) - pow(b2y, 2));

    // get q(0)
    q(0) = atan2(posF(2) * b2y + posF(1) * L, posF(1) * b2y - posF(2) * L);

    // get q(2)
    float temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
    if (temp > 1)
        temp = 1;
    if (temp < -1)
        temp = -1;
    q(2) = -(M_PI - acos(temp));

    // get q(1)
    float a1 = posF(1) * sin(q(0)) - posF(2) * cos(q(0));
    float a2 = posF(0);
    float m1 = b4z * sin(q(2));
    float m2 = b3z + b4z * cos(q(2));
    q(1) = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

    return q;
}

/**
 * @brief Jacobian matrix that maps joint velocity to foot velocity, expressed in ABAD frame
 * @param {Vec3} q: joint angle
 * @return {Mat3} Jacobian matrix
 */
Mat3 QuadrupedLeg::Jacobian(const Vec3 &q)
{
    Mat3 jaco;

    float l1 = _abadLink * _sideSign;
    float l2 = -_upperLink;
    float l3 = -_lowerLink;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    jaco(0, 0) = 0;
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jaco(0, 1) = l3 * c23 + l2 * c2;
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jaco(0, 2) = l3 * c23;
    jaco(1, 2) = l3 * s1 * s23;
    jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}

/**
 * @brief Forward kinematics derivative. Given joint velocity, calculate the velocity of foot
 * @param {Vec3} q: joint angle
 * @param {Vec3} dq: joint velocity
 * @return {Vec3} velocity of foot, expressed in ABAD frame
 */
Vec3 QuadrupedLeg::FKDerivative(const Vec3 &q, const Vec3 &dq)
{
    return Jacobian(q) * dq;
}

/**
 * @brief Inverse kinematics derivative. Given foot velocity, calculate the velocity of joint
 * @param {Vec3} q: joint angle
 * @param {Vec3} velF: foot velocity, expressed in ABAD frame
 * @return {Vec3} velocity of joint
 */
Vec3 QuadrupedLeg::IKDerivative(const Vec3 &q, const Vec3 &velF)
{
    return Jacobian(q).inverse() * velF;
}