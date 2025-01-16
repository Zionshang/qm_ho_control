#pragma once
#include "common/mathTypes.h"
#include "common/projectPath.h"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

class WholeBodyDynamics
{
public:
    WholeBodyDynamics();
    int getNq() { return _nq; }
    int getNv() { return _nv; }
    void calcZeroAccKinematics(const VecNq &q, const VecNv &v);
    void setCoMPosVel(const VecNq &q, const VecNv &v, Vector3d &posCoM, Vector3d &velCoM);
    void setFootJacob(const VecNq &q, int idLeg, Matrix6xd &J);
    void setFootdJdq(const VecNq &q, const VecNv &v, int idLeg, Vector6d &dJdq);
    void setMandC(const VecNq &q, const VecNv &v, MatrixXd &M, VecNv &C);
    void setBodyJacob(const VecNq &q, Matrix6xd &J);
    void setBodydJdq(const VecNq &q, const VecNv &v, Vector6d &dJdq);
    void setCoMJacob(const VecNq &q, Matrix3xd &J);
    void setCoMdJdq(const VecNq &q, const VecNv &v, Vector3d &dJdq);

    void setGripperJacob(const VecNq &q, Matrix6xd &J);
    void setGripperdJdq(const VecNq &q, const VecNv &v, Vector6d &dJdq);

private:
    pinocchio::Model _model;
    pinocchio::Data _data;
    int _idFeet[4];
    int _nq, _nv;
    int _idBody;

    int _idGripper;
};
