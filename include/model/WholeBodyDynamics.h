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
    void updateKinematics(const VecNq &q, const VecNv &v);
    void setCoMPosVel(const VecNq &q, const VecNv &v, Vec3 &posCoM, Vec3 &velCoM);
    void setFootJacob(const VecNq &q, int idLeg, Jacb &J);
    void setFootdJdq(const VecNq &q, const VecNv &v, int idLeg, Vec6 &dJdq);
    void setMandC(const VecNq &q, const VecNv &v, MatNv &M, VecNv &C);
    void setBodyJacob(const VecNq &q, Jacb &J);
    void setBodydJdq(const VecNq &q, const VecNv &v, Vec6 &dJdq);
    void setCoMJacob(const VecNq &q, JacbP &J);
    void setCoMdJdq(const VecNq &q, const VecNv &v, Vec3 &dJdq);

    void setGripperJacob(const VecNq &q, Jacb &J);
    void setGripperdJdq(const VecNq &q, const VecNv &v, Vec6 &dJdq);

private:
    pinocchio::Model _model;
    pinocchio::Data _data;
    int _idFeet[4];
    int _nq, _nv;
    int _idBody;

    int _idGripper;
};
