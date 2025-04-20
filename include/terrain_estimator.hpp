#pragma once
#include "common/types.hpp"

class TerrainEstimator
{
public:
    TerrainEstimator();

    void update(const Quaterniond &body_eular_angle,
                const Matrix34d &pos_feet2body,
                const Vector4i &contact_flag);
    // expressed in BODY frame
    Vector3d getGroundEulerAngleWrtBody() const { return ground_euler_angle; }

private:
    Matrix3d rotmat_body;
    Matrix34d pos_feet2body_body; // position of foot w.r.t body, expressed in BODY frame
    Vector3d ground_euler_angle;                                         // expressed in BODY frame
};
