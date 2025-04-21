#pragma once
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "common/types.hpp"

class Interpolator
{
public:
  explicit Interpolator(const pinocchio::Model &model);

  void interpolateConfiguration(const double delay, const double timestep,
                                const std::vector<VectorXd> &qs, VectorXd &q_interp);

  void interpolateState(const double delay, const double timestep,
                        const std::vector<VectorXd> &xs, VectorXd &x_interp);

  void interpolateLinear(const double delay, const double timestep,
                         const std::vector<VectorXd> &vs, VectorXd &v_interp);

private:
  pinocchio::Model model_;
};
