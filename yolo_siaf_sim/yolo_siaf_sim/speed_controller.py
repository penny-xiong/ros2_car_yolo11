import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class PedestrianSpeedController(Node):
    """Acceleration-limited longitudinal controller driven by detected range."""

    def __init__(self):
        super().__init__("pedestrian_speed_controller")
        self.declare_parameter("distance_topic", "/yolo_siaf/nearest_pedestrian_distance")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cruise_speed_mps", 1.50)
        self.declare_parameter("crawl_speed_mps", 0.15)
        self.declare_parameter("slow_distance_m", 30.0)
        self.declare_parameter("stop_distance_m", 5.0)
        self.declare_parameter("max_acceleration_mps2", 0.60)
        self.declare_parameter("max_deceleration_mps2", 1.50)
        self.declare_parameter("detection_timeout_s", 0.50)
        self.declare_parameter("control_rate_hz", 20.0)

        self.cruise = float(self.get_parameter("cruise_speed_mps").value)
        self.crawl = float(self.get_parameter("crawl_speed_mps").value)
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.max_accel = float(self.get_parameter("max_acceleration_mps2").value)
        self.max_decel = float(self.get_parameter("max_deceleration_mps2").value)
        self.timeout = float(self.get_parameter("detection_timeout_s").value)
        rate = float(self.get_parameter("control_rate_hz").value)

        if self.slow_distance <= self.stop_distance:
            raise ValueError("slow_distance_m must be greater than stop_distance_m")

        self.distance = math.inf
        self.last_detection_time = None
        self.current_speed = 0.0
        self.last_control_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("distance_topic").value),
            self._distance_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._control_callback)

    def _distance_callback(self, msg: Float32) -> None:
        if math.isfinite(msg.data) and msg.data >= 0.0:
            self.distance = float(msg.data)
            self.last_detection_time = self.get_clock().now()

    def _target_speed(self, now) -> float:
        if self.last_detection_time is None:
            return self.cruise
        age = (now - self.last_detection_time).nanoseconds * 1e-9
        if age > self.timeout:
            return self.cruise
        if self.distance <= self.stop_distance:
            return 0.0
        if self.distance >= self.slow_distance:
            return self.cruise
        ratio = (self.distance - self.stop_distance) / (
            self.slow_distance - self.stop_distance
        )
        return self.crawl + ratio * (self.cruise - self.crawl)

    def _control_callback(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds * 1e-9, 1e-3)
        self.last_control_time = now
        target = self._target_speed(now)

        if target > self.current_speed:
            self.current_speed = min(target, self.current_speed + self.max_accel * dt)
        else:
            self.current_speed = max(target, self.current_speed - self.max_decel * dt)

        cmd = Twist()
        cmd.linear.x = float(self.current_speed)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianSpeedController()
    try:
        rclpy.spin(node)
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
