#include "common/types.hpp"
#include "utils/cubic_spline.hpp"
#include "common/low_cmd.hpp"

class PositionController
{
public:
    PositionController(double dt);
    void updateToStandUp(const Matrix34d &joint_pos, LowCmd &low_cmd);
    void updateToLieDown(const Matrix34d &joint_pos, LowCmd &low_cmd);
    void updateArmToDefault(const Vector6d &joint_pos, LowCmd &low_cmd);

private:
    void setStandUp(const Matrix34d &current_pos);
    void setLieDown(const Matrix34d &current_pos);
    void setArmDefault(const Vector6d &current_pos);

    double time_;
    double period_;
    double dt_;

    bool stand_flag_;
    bool lie_flag_;
    bool arm_flag_;

    std::vector<CubicSpline> stand_spline0_, stand_spline1_;
    std::vector<CubicSpline> lie_spline_;
    std::vector<CubicSpline> arm_spline_;

    Matrix34d lie_joint_pos_;
    Matrix34d stand_joint_pos_;
    Vector6d default_arm_joint_pos_;
};