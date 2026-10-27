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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 动力学模型 + LQR 求解 + K(h) 调度 + 五杆足端 VMC + 差速底盘运动学库
add_library(leg_calc_model
  src/wheel_leg_model.cpp
  src/lqr.cpp
  src/foot_vmc.cpp
  src/diff_drive.cpp
)
target_include_directories(leg_calc_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(leg_calc_model PUBLIC Eigen3::Eigen)

# 模型 + LQR 演示节点
add_executable(model_demo_node src/model_demo_node.cpp)
target_link_libraries(model_demo_node leg_calc_model)
ament_target_dependencies(model_demo_node rclcpp std_msgs)

# 实时 LQR 平衡控制器：/imu/data -> 欧拉角 -> 状态 -> 控制量
add_executable(lqr_controller_node src/lqr_controller_node.cpp)
target_link_libraries(lqr_controller_node leg_calc_model)
ament_target_dependencies(lqr_controller_node
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
)

# K(h) 扫描 + 二次多项式拟合工具
add_executable(gain_table src/gain_table_tool.cpp)
target_link_libraries(gain_table leg_calc_model)
ament_target_dependencies(gain_table rclcpp)

# 足端虚拟弹簧阻尼（VMC）：/joint_states -> 足端 PD -> J^T -> /joint_target
add_executable(vmc_controller_node src/vmc_controller_node.cpp)
target_link_libraries(vmc_controller_node leg_calc_model)
ament_target_dependencies(vmc_controller_node rclcpp std_msgs sensor_msgs)

# 差速底盘：/cmd_vel -> 左右轮目标转速 -> /wheel_target
add_executable(diff_drive_node src/diff_drive_node.cpp)
target_link_libraries(diff_drive_node leg_calc_model)
ament_target_dependencies(diff_drive_node rclcpp std_msgs geometry_msgs)

install(
  TARGETS model_demo_node lqr_controller_node gain_table
          vmc_controller_node diff_drive_node
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

  ament_add_gtest(test_diff_drive test/test_diff_drive.cpp)
  target_link_libraries(test_diff_drive leg_calc_model)
endif()

ament_export_include_directories(include)
ament_package()
