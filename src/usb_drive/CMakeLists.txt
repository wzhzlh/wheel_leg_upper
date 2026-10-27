cmake_minimum_required(VERSION 3.8)
project(usb_drive)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# USB-CDC 传输库（libusb 异步收发 + 定长帧重组）
add_library(usb_drive_cdc
  src/cdc_trans.cpp
)
target_include_directories(usb_drive_cdc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBUSB_INCLUDE_DIRS}
)
target_link_directories(usb_drive_cdc PUBLIC ${LIBUSB_LIBRARY_DIRS})
target_link_libraries(usb_drive_cdc ${LIBUSB_LIBRARIES})
ament_target_dependencies(usb_drive_cdc rclcpp)

# 通信节点：控制指令下发 + 下位机状态上行
add_executable(usb_drive_node src/usb_drive_node.cpp)
target_link_libraries(usb_drive_node usb_drive_cdc)
ament_target_dependencies(usb_drive_node rclcpp std_msgs sensor_msgs nav_msgs)

install(
  TARGETS usb_drive_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
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
  ament_add_gtest(test_frame_assembler test/test_frame_assembler.cpp)
  target_link_libraries(test_frame_assembler usb_drive_cdc)
endif()

ament_export_include_directories(include)
ament_package()
