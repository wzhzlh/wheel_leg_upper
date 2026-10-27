#!/usr/bin/env python
"""YbImu 串口 IMU 驱动。

只发布标准 sensor_msgs/Imu（加速度、角速度、四元数姿态）到 /imu/data，
欧拉角转换与 LQR 状态组装由 leg_calc 的控制器节点完成。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from YbImuLib import YbImuSerial

FALLBACK_PORTS = ["/dev/myimu", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]


class YbImuDriver(Node):
    """YbImu 串口 IMU 驱动节点。"""

    def __init__(self):
        super().__init__("ybimu_driver")
        self.declare_parameter("port", "")
        self.declare_parameter("fallback_ports", FALLBACK_PORTS)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "imu/data")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("orientation_stddev", 0.01)

        self.robot = None
        self.frame_id = self.get_parameter("frame_id").value
        self.orientation_stddev = float(
            self.get_parameter("orientation_stddev").value)
        topic = self.get_parameter("topic").value

        self.imu_pub = self.create_publisher(Imu, topic, 10)
        self.try_open_serial()

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)
        # 串口未就绪时每 2 s 重试一次，打开成功后自动变成空操作
        self.retry_timer = self.create_timer(2.0, self.try_open_serial)

    def try_open_serial(self):
        """按 port 参数优先、再按 fallback_ports 顺序尝试打开串口。"""
        if self.robot is not None:
            return

        candidates = []
        port = self.get_parameter("port").value
        if port:
            candidates.append(port)
        for item in self.get_parameter("fallback_ports").value:
            if item not in candidates:
                candidates.append(item)

        for candidate in candidates:
            try:
                self.robot = YbImuSerial(candidate)
            except Exception as exc:  # noqa: BLE001 - 逐个端口试探
                self.get_logger().debug(f"打开 {candidate} 失败: {exc}")
                continue
            self.robot.create_receive_threading()
            self.get_logger().info(f"IMU 串口已打开: {candidate}")
            return

        self.get_logger().error(
            "无法打开 IMU 串口，请用 -p port:=/dev/ttyUSBx 指定后重试")

    def publish_imu(self):
        if self.robot is None:
            return
        try:
            ax, ay, az = self.robot.get_accelerometer_data()
            gx, gy, gz = self.robot.get_gyroscope_data()
            qw, qx, qy, qz = self.robot.get_imu_quaternion_data()
        except Exception as exc:  # noqa: BLE001 - 串口异常不应让节点退出
            self.get_logger().warn(
                f"读取 IMU 数据失败: {exc}", throttle_duration_sec=1.0)
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)

        variance = self.orientation_stddev ** 2
        imu.orientation.w = float(qw)
        imu.orientation.x = float(qx)
        imu.orientation.y = float(qy)
        imu.orientation.z = float(qz)
        imu.orientation_covariance[0] = variance
        imu.orientation_covariance[4] = variance
        imu.orientation_covariance[8] = variance

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = YbImuDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
