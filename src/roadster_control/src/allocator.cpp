#include "roadster_control/allocator.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <algorithm>
#include <utility>

namespace roadster_control
{

ThrustAllocator::ThrustAllocator(
  std::vector<NozzleGeom> nozzles, double force_scale, double torque_scale)
: nozzles_(std::move(nozzles))
{
  row_scale_ << force_scale, force_scale, force_scale, torque_scale, torque_scale, torque_scale;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ThrustAllocator::effectiveness(double feed_ratio) const
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> B(6, nozzles_.size());
  for (std::size_t i = 0; i < nozzles_.size(); ++i) {
    const auto & n = nozzles_[i];
    const double f = n.f_max * std::max(feed_ratio, 0.0);
    B.block<3, 1>(0, i) = n.dir * f;
    B.block<3, 1>(3, i) = n.r_com.cross(n.dir) * f;
  }
  return B;
}

AllocResult ThrustAllocator::solve(
  const Vec6 & w_des, const Vec6 & weights, double feed_ratio, double lambda,
  const Eigen::Matrix3d & force_frame) const
{
  const int n = static_cast<int>(nozzles_.size());
  AllocResult r;
  r.u = Eigen::VectorXd::Zero(n);
  r.achieved.setZero();
  if (n == 0 || feed_ratio <= 0.0) {
    return r;
  }
  const auto B = effectiveness(feed_ratio);
  Eigen::Matrix<double, 6, Eigen::Dynamic> Bf = B;
  Bf.topRows<3>() = force_frame * B.topRows<3>();
  // normalized, weighted rows: Bn = D Bf, wn = D w with D = diag(sqrt(W) / s)
  const Vec6 d = weights.cwiseMax(0.0).cwiseSqrt().cwiseQuotient(row_scale_);
  const Eigen::MatrixXd Bn = d.asDiagonal() * Bf;
  const Eigen::VectorXd wn = d.asDiagonal() * w_des;

  // 0.5 u' P u + q' u with P = 2 (Bn' Bn + lambda I), q = -2 Bn' wn
  const Eigen::MatrixXd P = 2.0 * (Bn.transpose() * Bn +
    lambda * Eigen::MatrixXd::Identity(n, n));
  const Eigen::VectorXd q = -2.0 * Bn.transpose() * wn;
  Eigen::SparseMatrix<double> Ps = P.sparseView();
  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n), hi = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd qv = q;

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  solver.settings()->setAbsoluteTolerance(1e-7);
  solver.settings()->setRelativeTolerance(1e-7);
  solver.settings()->setPolish(true);
  solver.data()->setNumberOfVariables(n);
  solver.data()->setNumberOfConstraints(n);
  bool ok = solver.data()->setHessianMatrix(Ps) && solver.data()->setGradient(qv) &&
    solver.data()->setLinearConstraintsMatrix(A) && solver.data()->setLowerBound(lo) &&
    solver.data()->setUpperBound(hi) && solver.initSolver();
  if (ok && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError &&
    solver.getStatus() == OsqpEigen::Status::Solved)
  {
    r.u = solver.getSolution().cwiseMax(0.0).cwiseMin(1.0);
    r.qp_ok = true;
  } else {
    // fallback: regularized least squares, clipped to the valve range
    r.u = (Bn.transpose() * Bn + lambda * Eigen::MatrixXd::Identity(n, n))
      .ldlt().solve(Bn.transpose() * wn).cwiseMax(0.0).cwiseMin(1.0);
  }
  r.achieved = B * r.u;
  return r;
}

}  // namespace roadster_control
