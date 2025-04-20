#include "terrain_estimator.hpp"

TerrainEstimator::TerrainEstimator()
{
    ground_euler_angle.setZero();
    rotmat_body.setIdentity();
}

void TerrainEstimator::update(const Quaterniond &body_eular_angle,
                              const Matrix34d &pos_feet2body,
                              const Vector4i &contact_flag)
{
    if (contact_flag[0] == true && contact_flag[1] == true && contact_flag[2] == true && contact_flag[3] == true)
    {
        rotmat_body = body_eular_angle.toRotationMatrix();
        for (int i = 0; i < 4; i++)
        {
            pos_feet2body_body.col(i) = rotmat_body.transpose() * pos_feet2body.col(i);
        }

        double x1 = pos_feet2body_body(0, 1); // FR x
        double x2 = pos_feet2body_body(0, 3); // HR x
        double x3 = pos_feet2body_body(0, 0); // FL x
        double x4 = pos_feet2body_body(0, 2); // HL x

        double y1 = pos_feet2body_body(1, 1); // FR y
        double y2 = pos_feet2body_body(1, 3); // HR y
        double y3 = pos_feet2body_body(1, 0); // FL y
        double y4 = pos_feet2body_body(1, 2); // HL y

        double z1 = pos_feet2body_body(2, 1); // FR z
        double z2 = pos_feet2body_body(2, 3); // HR z
        double z3 = pos_feet2body_body(2, 0); // FL z
        double z4 = pos_feet2body_body(2, 2); // HL z

        int sig_pitch, sig_roll;
        Eigen ::Matrix<double, 3, 3> M, mi;
        Vector3d b, a;
        Vector3d Nxoy, Ns, Ns_pitch, Ns_roll;

        b << x1 + x2 + x3 + x4, y1 + y2 + y3 + y4, z1 + z2 + z3 + z4;
        M << x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4, x1 * y1 + x2 * y2 + x3 * y3 + x4 * y4,
            x1 * z1 + x2 * z2 + x3 * z3 + x4 * z4, x1 * y1 + x2 * y2 + x3 * y3 + x4 * y4,
            y1 * y1 + y2 * y2 + y3 * y3 + y4 * y4, y1 * z1 + y2 * z2 + y3 * z3 + y4 * z4,
            x1 * z1 + x2 * z2 + x3 * z3 + x4 * z4, y1 * z1 + y2 * z2 + y3 * z3 + y4 * z4,
            z1 * z1 + z2 * z2 + z3 * z3 + z4 * z4;
        a = M.inverse() * b;
        mi = M.inverse() * M;

        Nxoy << 0, 0, 1;
        Ns << a(0), a(1), a(2);    // 空间任意平面的法向量  z = a1 + a2*x + a3*y
        Ns_pitch << a(0), 0, a(2); // Ns在xoz平面投影
        Ns_roll << 0, a(1), a(2);  // Ns在yoz平面投影
        double ground_pitch_ = M_PI - acos(Nxoy.dot(Ns_pitch) / (Nxoy.norm() * Ns_pitch.norm()));
        double ground_roll_ = M_PI - acos(Nxoy.dot(Ns_roll) / (Nxoy.norm() * Ns_roll.norm()));

        if (z1 - z2 + z3 - z4 > 0)
            sig_pitch = -1;
        else
            sig_pitch = 1;

        if (z1 + z2 - z3 - z4 > 0)
            sig_roll = -1;
        else
            sig_roll = 1;

        // relative to world frame
        ground_euler_angle(0) = 0.0;
        ground_euler_angle(1) = sig_pitch * ground_pitch_;
        ground_euler_angle(2) = sig_roll * ground_roll_;
    }
}
