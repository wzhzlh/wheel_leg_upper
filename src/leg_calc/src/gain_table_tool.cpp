/**
 * @brief 扫描质心高度 h、求解 LQR 增益，并做二次多项式拟合
 *
 * 输出可直接填进 lqr_params.yaml 的 k_coeffs：
 *   K_j(h) = c2*h^2 + c1*h + c0     （行 = k_x, k_theta, k_x_dot, k_theta_dot）
 *
 * 用法：
 *   ros2 run leg_calc gain_table --ros-args \
 *     -p h_min:=0.15 -p h_max:=0.45 -p samples:=31 \
 *     -p cart_mass:=1.2 -p pendulum_mass:=2.5 -p com_inertia:=0.05
 */

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg_calc/lqr.hpp"
#include "leg_calc/param_utils.hpp"

class GainTableTool : public rclcpp::Node
{
public:
  using Gain = Eigen::Matrix<double, 1, 4>;

  GainTableTool()
  : Node("leg_calc_gain_table")
  {
    leg_calc::declareModelParams(*this);
    leg_calc::declareLqrWeights(*this);
    declare_parameter("h_min", 0.15);
    declare_parameter("h_max", 0.45);
    declare_parameter("samples", 31);

    const leg_calc::ModelParams params = leg_calc::readModelParams(*this);
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    leg_calc::readLqrWeights(*this, &Q, &R);

    h_min_ = get_parameter("h_min").as_double();
    h_max_ = get_parameter("h_max").as_double();
    samples_ = static_cast<int>(get_parameter("samples").as_int());

    if (h_min_ <= 0.0 || h_max_ <= h_min_ || samples_ < 3) {
      throw std::invalid_argument("需满足 0 < h_min < h_max 且 samples >= 3");
    }

    schedule_.setModelParams(params);
    schedule_.setMode(leg_calc::GainMode::kOnlineLqr);
    schedule_.setWeights(Q, R);
    schedule_.setHeightLimits(h_min_, h_max_);
  }

  int run()
  {
    std::printf("h[m],k_x,k_theta,k_x_dot,k_theta_dot\n");

    Eigen::MatrixXd basis(static_cast<Eigen::Index>(samples_), 3);
    Eigen::MatrixXd gains(static_cast<Eigen::Index>(samples_), 4);

    for (int i = 0; i < samples_; ++i) {
      const double h = h_min_ + (h_max_ - h_min_) * static_cast<double>(i) /
        static_cast<double>(samples_ - 1);
      const Gain k = schedule_.solveLqrAt(h);
      if (!schedule_.lastSolveOk()) {
        std::fprintf(stderr, "h=%.4f 处 LQR 求解失败\n", h);
        return 1;
      }
      basis.row(i) << h * h, h, 1.0;
      gains.row(i) = k;
      std::printf(
        "%.4f,%.6f,%.6f,%.6f,%.6f\n", h, k(0, 0), k(0, 1), k(0, 2), k(0, 3));
    }

    // 最小二乘：gains(12x4) ≈ basis(12x3) * coeffs^T  =>  coeffs 为 4x3
    const Eigen::Matrix<double, 4, 3> coeffs =
      (basis.householderQr().solve(gains)).transpose();
    const Eigen::MatrixXd residual = basis * coeffs.transpose() - gains;

    std::printf("\n# 二次拟合 K_j(h) = c2*h^2 + c1*h + c0，最大残差 %.6f\n",
      residual.cwiseAbs().maxCoeff());
    std::printf("# 填入 lqr_params.yaml 的 k_coeffs（行: k_x / k_theta / k_x_dot / k_theta_dot）:\n");
    std::printf("    k_coeffs: [\n");
    static const char * labels[4] = {"k_x        ", "k_theta    ", "k_x_dot    ", "k_theta_dot"};
    for (int j = 0; j < 4; ++j) {
      std::printf(
        "      %+.6f, %+.6f, %+.6f,  # %s\n",
        coeffs(j, 0), coeffs(j, 1), coeffs(j, 2), labels[j]);
    }
    std::printf("    ]\n");
    return 0;
  }

private:
  leg_calc::GainSchedule schedule_;
  double h_min_{0.15};
  double h_max_{0.45};
  int samples_{31};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GainTableTool>();
  const int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
