// allocator.hpp - cold-gas thrust allocation (plan 5.7).
//
// Valve openings u in [0, 1]^N for a requested body wrench w = [F; tau]:
//   min  sum_k W_k ((B u - w)_k / s_k)^2 + lambda ||u||^2   s.t. 0 <= u <= 1
// B's column i is f_max_i * feed_ratio * [d_i; r_i x d_i] (r_i from the COM),
// s_k normalizes rows (forces by m g, torques by m g * 1 m) so the weights W_k
// are dimensionless priorities. Solved with OSQP (the MPC's solver); when OSQP
// fails, a clipped least-squares solution is returned instead.
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace roadster_control
{

using Vec6 = Eigen::Matrix<double, 6, 1>;

struct NozzleGeom
{
  std::string name;
  Eigen::Vector3d r_com;   // nozzle position relative to the COM, body frame [m]
  Eigen::Vector3d dir;     // unit force direction on the car, body frame
  double f_max = 0.0;      // at feed_ratio = 1 [N]
};

struct AllocResult
{
  Eigen::VectorXd u;       // valve commands, 0..1
  Vec6 achieved;           // B u
  bool qp_ok = false;      // false: clipped least-squares fallback was used
};

class ThrustAllocator
{
public:
  ThrustAllocator(std::vector<NozzleGeom> nozzles, double force_scale, double torque_scale);

  // feed_ratio scales every f_max (tank feed / reference pressure).
  // force_frame rotates the force rows before they are weighted: pass the
  // body-to-world rotation to weight (and request) forces in WORLD axes, e.g.
  // to penalize horizontal drift during a flip whatever the body attitude.
  // w_des forces are then world-frame; torques stay body-frame. The returned
  // `achieved` is always the body-frame wrench B u.
  AllocResult solve(const Vec6 & w_des, const Vec6 & weights, double feed_ratio,
    double lambda, const Eigen::Matrix3d & force_frame = Eigen::Matrix3d::Identity()) const;

  Eigen::Matrix<double, 6, Eigen::Dynamic> effectiveness(double feed_ratio) const;
  std::size_t size() const {return nozzles_.size();}
  const std::vector<NozzleGeom> & nozzles() const {return nozzles_;}

private:
  std::vector<NozzleGeom> nozzles_;
  Vec6 row_scale_;
};

}  // namespace roadster_control
