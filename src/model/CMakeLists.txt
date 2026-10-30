cmake_minimum_required(VERSION 3.8)
project(model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 机器人描述与仿真资源（无编译目标，纯资源包；launch 统一在 launch_pack）
# meshes 必须一并安装，否则 URDF 里的 package://model/meshes/*.STL 找不到文件
install(DIRECTORY model config meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
