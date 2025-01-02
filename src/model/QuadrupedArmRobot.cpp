#include "model/QuadrupedArmRobot.h"
#include <iostream>

QuadrupedArmRobot::QuadrupedArmRobot()
{
    _abadOffsetX = 0.2407;
    _abadOffsetY = 0.051;
    _Legs[0] = new QuadrupedLeg(0);
    _Legs[1] = new QuadrupedLeg(1);
    _Legs[2] = new QuadrupedLeg(2);
    _Legs[3] = new QuadrupedLeg(3);

    _posA2B << _abadOffsetX, -_abadOffsetX, _abadOffsetX, -_abadOffsetX, // clang-format off
              -_abadOffsetY, -_abadOffsetY, _abadOffsetY,  _abadOffsetY,
                          0,             0,            0,             0; // clang-format on

    _arm = new Arm();
    _posS2B << 0.2535, 0, 0.056;
}

QuadrupedArmRobot::~QuadrupedArmRobot()
{
    delete _Legs[0];
    delete _Legs[1];
    delete _Legs[2];
    delete _Legs[3];
    delete _arm;
}

/**
 * @brief calculate foot position of one leg, numbered with ID
 * @param {Vec3} &q: joint angle of one leg, numbered with ID
 * @param {int} id： the ID of leg
 * @return {Vec3}: foot position w.r.t. BODY frame, expressed in BODY frame
 */
Vec3 QuadrupedArmRobot::calFootPos(const Vec3 &q, int id)
{
    return _posA2B.col(id) + _Legs[id]->FK(q);
}

/**
 * @brief get foot position of all legs
 * @param {Vec34} &q: joint angle of all legs
 * @return {Vec34} four feet position w.r.t. BODY frame, expressed in BODY frame
 */
Vec34 QuadrupedArmRobot::calFeetPos(const Vec34 &q)
{
    Vec34 feetPos;
    for (int i = 0; i < 4; i++)
        feetPos.col(i) = calFootPos(q.col(i), i);
    return feetPos;
}

/**
 * @brief calFootPos foot velocity of one leg, numbered with ID
 * @param {Vec3} &q: joint angle of one leg
 * @param {Vec3} &dq: joint velocity of one leg
 * @param {int} id: the ID of leg
 * @return {Vec3} foot velocity of ID leg, expressed in BODY frame
 */
Vec3 QuadrupedArmRobot::calFootVel(const Vec3 &q, const Vec3 &dq, int id)
{
    return _Legs[id]->FKDerivative(q, dq);
}

/**
 * @brief calFootPos foot velocity of all legs
 * @param {Vec34} &q: joint angle of all legs
 * @param {Vec34} &dq: joint velocity of all legs
 * @return {Vec34} foot velocity of all legs, expressed in BODY frame
 */
Vec34 QuadrupedArmRobot::calFeetVel(const Vec34 &q, const Vec34 &dq)
{
    Vec34 feetVel;
    for (int i = 0; i < 4; i++)
        feetVel.col(i) = _Legs[i]->FKDerivative(q.col(i), dq.col(i));
    return feetVel;
}

Vec34 QuadrupedArmRobot::calQLeg(const Vec34 &posF2B_B)
{
    Vec34 q;
    for (int i = 0; i < 4; i++)
        q.col(i) = _Legs[i]->IK(posF2B_B.col(i) - _posA2B.col(i));
    return q;
}
Vec34 QuadrupedArmRobot::calDqLeg(const Vec34 &q, const Vec34 &velF2B_B)
{
    Vec34 dq;
    for (int i = 0; i < 4; i++)
        dq.col(i) = _Legs[i]->IKDerivative(q.col(i), velF2B_B.col(i));
    return q;
}

/**
 * @brief calculate all state (postion, quaternion, velocity, angular velocity) of Gripper
 * @param {Vec6} &q: current joint position of arm
 * @param {Vec6} &dq: current joint velocity of arm
 * @param {Vec3} &posG: position of GRIPER w.r.t. BODY, expressed in BODY frame
 * @param {Quat} &quatG: quaternion of GRIPER frame w.r.t. BODY frame
 * @param {Vec3} &velG: velocity of GRIPER, expressed in BODY frame
 * @param {Vec3} &angVelG: angular velocity of GRIPER, expressed in BODY frame
 */
void QuadrupedArmRobot::setAllGriperState(const Vec6 &q, Vec3 &posG, Quat &quatG,
                                          const Vec6 &dq, Vec3 &velG, Vec3 &angVelG)
{
    Vec3 posG2S_S;
    _arm->setAllGriperState(q, posG2S_S, quatG, dq, velG, angVelG);
    posG = _posS2B + posG2S_S;
}