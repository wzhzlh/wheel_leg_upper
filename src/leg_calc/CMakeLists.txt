cmake_minimum_required(VERSION 3.8)
project(leg_calc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 控制核心：动力学模型 + LQR + 五杆足端 VMC + 安全状态机
# 全部不依赖 ROS，可直接被单元测试调用
add_library(leg_calc_model
  src/wheel_leg_model.cpp
  src/lqr.cpp
  src/foot_vmc.cpp
  src/leg_control_core.cpp
)
target_include_directories(leg_calc_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(leg_calc_model PUBLIC Eigen3::Eigen)

# 仿真和实机共用的控制节点
# 500 Hz 统一控制环：状态机 + Pitch LQR + 双侧 VMC + 轮子分配（ROS 外壳 + IO）
add_executable(leg_control_node
  src/leg_control_node.cpp
  src/leg_control_io.cpp
)
target_link_libraries(leg_control_node leg_calc_model)
ament_target_dependencies(
  leg_control_node rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2_ros)

# 上层指令参数服务：rqt / ros2 param → /robot_command（保持控制节点接口不变）
add_executable(command_param_node src/command_param_node.cpp)
target_include_directories(command_param_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(command_param_node rclcpp std_msgs)

# 仿真 I/O 桥（仅仿真用）：补 IMU 姿态 + joint_target/wheel_target → effort 控制器命令
add_executable(sim_bridge_node src/sim_bridge_node.cpp)
ament_target_dependencies(sim_bridge_node rclcpp std_msgs sensor_msgs geometry_msgs)

install(
  TARGETS leg_control_node command_param_node sim_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel_leg_model test/test_wheel_leg_model.cpp)
  target_link_libraries(test_wheel_leg_model leg_calc_model)

  ament_add_gtest(test_foot_vmc test/test_foot_vmc.cpp)
  target_link_libraries(test_foot_vmc leg_calc_model)

  # 状态机 / 安全监督：不依赖 ROS，可直接用假快照驱动
  ament_add_gtest(test_control_state test/test_control_state.cpp)
  target_link_libraries(test_control_state leg_calc_model)

  # 控制核心：状态机门控、VMC 力矩、轮子分配、命令/传感器超时
  ament_add_gtest(test_leg_control_core test/test_leg_control_core.cpp)
  target_link_libraries(test_leg_control_core leg_calc_model)

  ament_add_gtest(test_leg_control_io test/test_leg_control_io.cpp src/leg_control_io.cpp)
  target_link_libraries(test_leg_control_io leg_calc_model)
  ament_target_dependencies(test_leg_control_io rclcpp std_msgs sensor_msgs nav_msgs)
endif()

ament_export_include_directories(include)
ament_package()
