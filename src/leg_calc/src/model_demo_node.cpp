/**
 * @brief 轮-摆倒立摆模型 + LQR 演示节点
 *
 * 流程（与纸上推导一一对应）：
 *   1. 读取物理参数，构造拉格朗日非线性模型（massMatrix / biasVector）
 *   2. 在直立平衡点做解析小角度线性化，得到状态空间 (A, B)
 *   3. 求解连续时间 Riccati 方程得到 LQR 增益 K
 *   4. 用非线性模型（RK4）闭环仿真，验证线性控制器在原非线性系统上有效
 *
 * 运行示例：
 *   ros2 run leg_calc model_demo_node --ros-args \
 *     -p cart_mass:=1.2 -p pendulum_mass:=2.5 -p com_length:=0.28 \
 *     -p com_inertia:=0.05 -p wheel_radius:=0.065 -p wheel_inertia:=0.0008 \
 *     -p input_mode:=wheel_torque -p initial_state:="[0.0, 0.2, 0.0, 0.0]"
 *
 * 话题：
 *   ~/state    (std_msgs/Float64MultiArray) [x, theta, x_dot, theta_dot]
 *   ~/control  (std_msgs/Float64MultiArray) [u]
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg_calc/lqr.hpp"
#include "leg_calc/param_utils.hpp"
#include "leg_calc/wheel_leg_model.hpp"

namespace
{

std::string to_string(const std::string & name, const Eigen::MatrixXd & mat)
{
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  std::ostringstream os;
  os << name << " =\n" << mat.format(fmt);
  return os.str();
}

}  // namespace

class ModelDemoNode : public rclcpp::Node
{
public:
  ModelDemoNode()
  : Node("leg_calc_model_demo")
  {
    leg_calc::declareModelParams(*this);
    leg_calc::declareLqrWeights(*this, {10.0, 200.0, 1.0, 1.0});
    declare_parameter("dt", 0.001);
    declare_parameter("sim_time", 5.0);

    const leg_calc::ModelParams params = leg_calc::readModelParams(*this);
    model_.setParams(params);

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    leg_calc::readLqrWeights(*this, &Q, &R);
    dt_ = get_parameter("dt").as_double();
    sim_time_ = get_parameter("sim_time").as_double();

    const std::vector<double> x0 = declare_parameter(
      "initial_state", std::vector<double>{0.0, 0.2, 0.0, 0.0});
    const std::vector<double> xref = declare_parameter(
      "reference_state", std::vector<double>{0.0, 0.0, 0.0, 0.0});

    if (x0.size() != static_cast<size_t>(leg_calc::kStateDim) ||
      xref.size() != static_cast<size_t>(leg_calc::kStateDim))
    {
      throw std::invalid_argument(
              "initial_state / reference_state must each contain 4 elements");
    }
    for (int i = 0; i < leg_calc::kStateDim; ++i) {
      state_(i) = x0[static_cast<size_t>(i)];
      x_ref_(i) = xref[static_cast<size_t>(i)];
    }

    // ---------- 1. 小角度线性化：x_dot = A x + B u ----------
    leg_calc::WheelLegModel::MatA A;
    leg_calc::WheelLegModel::MatB B;
    model_.linearize(&A, &B);

    // 用数值雅可比交叉验证解析线性化
    leg_calc::WheelLegModel::MatA A_num;
    leg_calc::WheelLegModel::MatB B_num;
    model_.linearizeNumeric(leg_calc::WheelLegModel::State::Zero(), 0.0, 1e-6, &A_num, &B_num);
    const double lin_error = std::max((A - A_num).cwiseAbs().maxCoeff(),
        (B - B_num).cwiseAbs().maxCoeff());

    // ---------- 2. LQR 求解 ----------
    const leg_calc::LqrSolution lqr =
      leg_calc::solveContinuousLqr(A, B, Q, R);

    RCLCPP_INFO(get_logger(), "模型参数: M=%.3f kg, m=%.3f kg, h=%.3f m, I=%.4f kg*m^2, "
      "r=%.3f m, Jw=%.5f kg*m^2, 输入=%s, 等效平动质量=%.3f kg",
      params.cart_mass, params.pendulum_mass, params.com_length, params.com_inertia,
      params.wheel_radius, params.wheel_inertia,
      params.input_mode == leg_calc::InputMode::kForce ? "force" : "wheel_torque",
      model_.effectiveCartMass());
    RCLCPP_INFO(get_logger(), "\n%s", to_string("A", A).c_str());
    RCLCPP_INFO(get_logger(), "\n%s", to_string("B", B).c_str());
    RCLCPP_INFO(get_logger(), "解析/数值线性化最大偏差: %.3e", lin_error);

    if (!lqr.valid || !lqr.isStable()) {
      RCLCPP_ERROR(get_logger(), "LQR 求解失败（Q/R 需满足可控性、正定性要求）");
      throw std::runtime_error("LQR solve failed");
    }

    gain_ = lqr.gain;
    RCLCPP_INFO(get_logger(), "\n%s", to_string("K", gain_).c_str());
    RCLCPP_INFO(get_logger(), "Riccati 残差: %.3e", lqr.riccati_residual);
    RCLCPP_INFO(get_logger(), "闭环极点: %s",
      to_string("eig(A-BK)", lqr.closed_loop_eigenvalues_real).c_str());

    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);
    control_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/control", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&ModelDemoNode::step, this));

    RCLCPP_INFO(get_logger(), "开始闭环仿真: dt=%.4f s, 总时长=%.2f s", dt_, sim_time_);
  }

private:
  void step()
  {
    // u = -K (x - x_ref)
    const double u = -(gain_ * (state_ - x_ref_))(0, 0);

    // 用非线性模型积分，验证线性 LQR 在原系统上的有效性
    state_ = model_.integrateRK4(state_, u, dt_);
    sim_time_elapsed_ += dt_;

    std_msgs::msg::Float64MultiArray state_msg;
    state_msg.data = {state_(0), state_(1), state_(2), state_(3)};
    state_pub_->publish(state_msg);

    std_msgs::msg::Float64MultiArray control_msg;
    control_msg.data = {u};
    control_pub_->publish(control_msg);

    if (sim_time_elapsed_ + 1e-12 >= sim_time_) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(),
        "仿真结束: x=%.4f m, theta=%.4f rad, x_dot=%.4f m/s, theta_dot=%.4f rad/s, u=%.4f",
        state_(0), state_(1), state_(2), state_(3), u);
    }
  }

  leg_calc::WheelLegModel model_;
  leg_calc::WheelLegModel::State state_{leg_calc::WheelLegModel::State::Zero()};
  leg_calc::WheelLegModel::State x_ref_{leg_calc::WheelLegModel::State::Zero()};

  double dt_{0.001};
  double sim_time_{5.0};
  double sim_time_elapsed_{0.0};

  Eigen::MatrixXd gain_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModelDemoNode>());
  rclcpp::shutdown();
  return 0;
}
