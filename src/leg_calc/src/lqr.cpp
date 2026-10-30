#include "leg_calc/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace leg_calc
{
namespace
{

/// h 变化超过该阈值才重新求解 LQR（避免每周期都做特征分解）
constexpr double kCacheTolerance = 1e-4;

}  // namespace

// ============================ Riccati 求解 ============================

bool LqrSolution::isStable() const
{
  if (!valid) {
    return false;
  }
  return (closed_loop_eigenvalues_real.array() < 0.0).all();
}

LqrSolution solveContinuousLqr(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
{
  LqrSolution result;

  const Eigen::Index n = A.rows();
  const Eigen::Index m = B.cols();
  if (A.cols() != n || B.rows() != n || Q.rows() != n || Q.cols() != n ||
    R.rows() != m || R.cols() != m)
  {
    return result;
  }
  if (n == 0) {
    return result;
  }
  if (!A.allFinite() || !B.allFinite() || !Q.allFinite() || !R.allFinite() ||
    !R.isApprox(R.transpose(), 1e-10))
  {
    return result;
  }
  const Eigen::LDLT<Eigen::MatrixXd> r_ldlt(R);
  if (r_ldlt.info() != Eigen::Success ||
    !(r_ldlt.vectorD().array() > 0.0).all())
  {
    return result;
  }

  const Eigen::MatrixXd R_inv = R.inverse();
  const Eigen::MatrixXd B_Rinv_Bt = B * R_inv * B.transpose();

  // 哈密顿矩阵
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -B_Rinv_Bt;
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = -A.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> solver(H, true);
  if (solver.info() != Eigen::Success) {
    return result;
  }

  // 取实部为负（稳定）的 n 个特征向量
  std::vector<Eigen::Index> stable_indices;
  stable_indices.reserve(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < H.rows(); ++i) {
    if (solver.eigenvalues()(i).real() < 0.0) {
      stable_indices.push_back(i);
    }
  }
  if (static_cast<Eigen::Index>(stable_indices.size()) != n) {
    return result;
  }

  Eigen::MatrixXcd V(2 * n, n);
  for (Eigen::Index j = 0; j < n; ++j) {
    V.col(j) = solver.eigenvectors().col(stable_indices[static_cast<size_t>(j)]);
  }
  const Eigen::MatrixXcd V1 = V.topRows(n);
  const Eigen::MatrixXcd V2 = V.bottomRows(n);

  // P = V2 * V1^{-1}
  // 为避开直接求逆，先解 V1^T * X = V2^T 得 X = V1^{-T} * V2^T，再转置得 P
  const Eigen::MatrixXcd X = V1.transpose().fullPivLu().solve(V2.transpose());
  const Eigen::MatrixXcd P_complex = X.transpose();
  Eigen::MatrixXd P = P_complex.real();
  P = 0.5 * (P + P.transpose()).eval();  // 消除数值误差导致的非对称

  if (!P.allFinite()) {
    return result;
  }

  const Eigen::MatrixXd K = R_inv * B.transpose() * P;
  const Eigen::MatrixXd care =
    A.transpose() * P + P * A - P * B_Rinv_Bt * P + Q;

  result.riccati = P;
  result.gain = K;
  result.riccati_residual = care.norm();

  Eigen::EigenSolver<Eigen::MatrixXd> closed_loop(A - B * K, false);
  result.closed_loop_eigenvalues_real = closed_loop.eigenvalues().real();
  result.closed_loop_eigenvalues_imag = closed_loop.eigenvalues().imag();

  // P 正定（对称阵用 LDLT 判断）
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(P);
  const bool positive_definite =
    ldlt.info() == Eigen::Success && (ldlt.vectorD().array() > 0.0).all();
  const bool symmetric = (P - P.transpose()).norm() <
    1e-8 * std::max(1.0, P.norm());

  result.valid = positive_definite && symmetric &&
    result.riccati_residual < 1e-6 * std::max(1.0, (A.transpose() * P + P * A).norm());

  return result;
}

// ======================= 腿部几何 + K(h) 调度 =======================

double LegGeometry::comHeight(double hip, double knee) const
{
  return thigh * std::cos(hip) + shank * std::cos(hip + knee) + com_offset;
}

double LegGeometry::comHeight(const std::array<double, 4> & joints) const
{
  // joints = [左髋, 左膝, 右髋, 右膝]
  return 0.5 * (comHeight(joints[0], joints[1]) + comHeight(joints[2], joints[3]));
}

GainSchedule::GainSchedule(const ModelParams & params, GainMode mode)
: params_(params), mode_(mode)
{
}

void GainSchedule::setModelParams(const ModelParams & params)
{
  params_ = params;
  cache_valid_ = false;
}

void GainSchedule::setMode(GainMode mode)
{
  mode_ = mode;
  cache_valid_ = false;
}

void GainSchedule::setWeights(const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
{
  if (Q.rows() != kStateDim || Q.cols() != kStateDim) {
    throw std::invalid_argument("Q 必须是 4x4");
  }
  if (R.rows() != 1 || R.cols() != 1) {
    throw std::invalid_argument("R 必须是 1x1");
  }
  if (!Q.allFinite() || !R.allFinite() || !(R(0, 0) > 0.0)) {
    throw std::invalid_argument("Q/R 必须有限，且 R 必须为正");
  }
  Q_ = Q;
  R_ = R;
  cache_valid_ = false;
}

void GainSchedule::setQuadraticCoefficients(const Eigen::Matrix<double, 4, 3> & coeffs)
{
  if (!coeffs.allFinite()) {
    throw std::invalid_argument("k_coeffs 必须全部为有限值");
  }
  coeffs_ = coeffs;
  // 全零视为“未提供系数”，退回在线 LQR
  coeffs_valid_ = coeffs_.cwiseAbs().maxCoeff() > 0.0;
  cache_valid_ = false;
}

void GainSchedule::setHeightLimits(double h_min, double h_max)
{
  if (!(h_min > 0.0) || !(h_max > h_min)) {
    throw std::invalid_argument("高度区间必须满足 0 < h_min < h_max");
  }
  h_min_ = h_min;
  h_max_ = h_max;
}

GainSchedule::Gain GainSchedule::gainAt(double com_height)
{
  const double h = std::clamp(
    std::isfinite(com_height) ? com_height : params_.com_length, h_min_, h_max_);

  if (mode_ == GainMode::kQuadratic && coeffs_valid_) {
    Gain gain;
    for (int j = 0; j < 4; ++j) {
      gain(0, j) = coeffs_(j, 0) * h * h + coeffs_(j, 1) * h + coeffs_(j, 2);
    }
    last_solve_ok_ = gain.allFinite();
    return gain;
  }

  if (cache_valid_ && std::abs(h - cached_h_) < kCacheTolerance) {
    return cached_gain_;
  }

  const Gain gain = solveLqrAt(h);
  if (last_solve_ok_) {
    cached_gain_ = gain;
    cached_h_ = h;
    cache_valid_ = true;
  }
  return gain;
}

GainSchedule::Gain GainSchedule::solveLqrAt(double com_height)
{
  ModelParams params = params_;
  params.com_length = com_height;  // 高度变化等价于“摆杆长度”变化

  WheelLegModel model(params);
  WheelLegModel::MatA A;
  WheelLegModel::MatB B;
  model.linearize(&A, &B);

  const LqrSolution solution = solveContinuousLqr(A, B, Q_, R_);
  last_solve_ok_ = solution.valid && solution.isStable();
  if (!last_solve_ok_) {
    // 求解失败时保留上一次可用增益（首次失败则返回全零并置标志位）
    return cache_valid_ ? cached_gain_ : Gain::Zero();
  }
  return solution.gain;
}

}  // namespace leg_calc
