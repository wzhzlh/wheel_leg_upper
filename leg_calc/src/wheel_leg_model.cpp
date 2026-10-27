#include "leg_calc/wheel_leg_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace leg_calc
{

WheelLegModel::WheelLegModel(const ModelParams & params)
{
  setParams(params);
}

void WheelLegModel::setParams(const ModelParams & params)
{
  if (params.cart_mass <= 0.0) {
    throw std::invalid_argument("cart_mass must be positive");
  }
  if (params.pendulum_mass <= 0.0) {
    throw std::invalid_argument("pendulum_mass must be positive");
  }
  if (params.com_length <= 0.0) {
    throw std::invalid_argument("com_length must be positive");
  }
  if (params.com_inertia < 0.0) {
    throw std::invalid_argument("com_inertia must be non-negative");
  }
  if (params.gravity <= 0.0) {
    throw std::invalid_argument("gravity must be positive");
  }
  if (params.wheel_radius <= 0.0) {
    throw std::invalid_argument("wheel_radius must be positive");
  }
  if (params.wheel_inertia < 0.0) {
    throw std::invalid_argument("wheel_inertia must be non-negative");
  }
  params_ = params;
}

double WheelLegModel::effectiveCartMass() const
{
  // 轮子转动惯量按无滑动假设折算为平动质量：Jw * (x_dot / r)^2 / x_dot^2
  const double r = params_.wheel_radius;
  return params_.cart_mass + params_.wheel_inertia / (r * r);
}

Eigen::Matrix2d WheelLegModel::massMatrix(double theta) const
{
  const double M = effectiveCartMass();
  const double m = params_.pendulum_mass;
  const double h = params_.com_length;
  const double I = params_.com_inertia;

  Eigen::Matrix2d D;
  D << M + m, m * h * std::cos(theta),
    m * h * std::cos(theta), I + m * h * h;
  return D;
}

Eigen::Vector2d WheelLegModel::biasVector(double theta, double theta_dot) const
{
  const double m = params_.pendulum_mass;
  const double h = params_.com_length;
  const double g = params_.gravity;

  Eigen::Vector2d h_vec;
  h_vec << -m * h * theta_dot * theta_dot * std::sin(theta),
    -m * g * h * std::sin(theta);
  return h_vec;
}

Eigen::Vector2d WheelLegModel::inputMatrix() const
{
  Eigen::Vector2d Bq = Eigen::Vector2d::Zero();
  if (params_.input_mode == InputMode::kForce) {
    Bq(0) = 1.0;
  } else {
    Bq(0) = 1.0 / params_.wheel_radius;
  }
  return Bq;
}

Eigen::Vector2d WheelLegModel::jointAcceleration(double theta, double theta_dot, double u) const
{
  const Eigen::Vector2d rhs = inputMatrix() * u - biasVector(theta, theta_dot);
  return massMatrix(theta).ldlt().solve(rhs);
}

WheelLegModel::State WheelLegModel::dynamics(const State & state, double u) const
{
  const Eigen::Vector2d q_ddot =
    jointAcceleration(state(kTheta), state(kThetadot), u);

  State x_dot;
  x_dot(kX) = state(kXdot);
  x_dot(kTheta) = state(kThetadot);
  x_dot(kXdot) = q_ddot(0);
  x_dot(kThetadot) = q_ddot(1);
  return x_dot;
}

WheelLegModel::State WheelLegModel::integrateRK4(const State & state, double u, double dt) const
{
  const State k1 = dynamics(state, u);
  const State k2 = dynamics(state + 0.5 * dt * k1, u);
  const State k3 = dynamics(state + 0.5 * dt * k2, u);
  const State k4 = dynamics(state + dt * k3, u);
  return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void WheelLegModel::linearize(MatA * A, MatB * B) const
{
  const double M = effectiveCartMass();
  const double m = params_.pendulum_mass;
  const double h = params_.com_length;
  const double I = params_.com_inertia;
  const double g = params_.gravity;

  // D0 = D(0)：theta = 0 时 cos(theta) = 1
  Eigen::Matrix2d D0;
  D0 << M + m, m * h,
    m * h, I + m * h * h;
  const Eigen::Matrix2d D0_inv = D0.inverse();

  // dh/dtheta 在 (theta=0, theta_dot=0) 处 = [0, -m*g*h]^T
  const Eigen::Vector2d dh_dtheta(0.0, -m * g * h);

  // q_ddot = -D0^{-1} * (dh/dtheta) * theta + D0^{-1} * Bq * u
  A->setZero();
  (*A)(kX, kXdot) = 1.0;
  (*A)(kTheta, kThetadot) = 1.0;
  A->block<2, 1>(kXdot, kTheta) = -D0_inv * dh_dtheta;

  B->setZero();
  B->segment<2>(kXdot) = D0_inv * inputMatrix();
}

void WheelLegModel::linearizeNumeric(
  const State & x0, double u0, double eps, MatA * A, MatB * B) const
{
  const State f0 = dynamics(x0, u0);

  for (int j = 0; j < kStateDim; ++j) {
    const double dx = eps * std::max(1.0, std::abs(x0(j)));
    State xp = x0;
    State xm = x0;
    xp(j) += dx;
    xm(j) -= dx;
    A->col(j) = (dynamics(xp, u0) - dynamics(xm, u0)) / (2.0 * dx);
  }

  const double du = eps * std::max(1.0, std::abs(u0));
  *B = (dynamics(x0, u0 + du) - dynamics(x0, u0 - du)) / (2.0 * du);

  (void)f0;
}

double WheelLegModel::totalEnergy(const State & state) const
{
  const double M = effectiveCartMass();
  const double m = params_.pendulum_mass;
  const double h = params_.com_length;
  const double I = params_.com_inertia;
  const double g = params_.gravity;

  const double x_dot = state(kXdot);
  const double theta = state(kTheta);
  const double theta_dot = state(kThetadot);

  const double T = 0.5 * M * x_dot * x_dot +
    0.5 * m * (x_dot * x_dot + 2.0 * h * x_dot * theta_dot * std::cos(theta) +
    h * h * theta_dot * theta_dot) +
    0.5 * I * theta_dot * theta_dot;
  const double V = m * g * h * std::cos(theta);
  return T + V;
}

}  // namespace leg_calc
