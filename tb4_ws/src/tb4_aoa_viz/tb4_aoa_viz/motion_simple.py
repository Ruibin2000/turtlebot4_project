#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(x, y, z, w) -> float:
    """把四元数转换成 yaw (绕Z轴旋转角)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    """把角度规一到 (-pi, pi]"""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0:
        a += 2.0 * math.pi
    return a - math.pi


class MotionSimple(Node):
    """
    简单的运动节点：
    - 固定频率往 /cmd_vel_unstamped 发当前 Twist
    - 订阅 /odom，保存当前位姿 (x, y, yaw)
    """

    def __init__(self,
                 max_linear: float = 0.2,
                 max_angular: float = 0.5,
                 cmd_topic: str = '/cmd_vel_unstamped',
                 odom_topic: str = '/odom',
                 rate: float = 10.0):
        super().__init__('motion_simple')

        self.max_linear = max_linear
        self.max_angular = max_angular

        # 当前速度指令
        self._cmd = Twist()

        # Publisher: 和你命令行一致，用 /cmd_vel_unstamped
        self._pub = self.create_publisher(Twist, cmd_topic, 10)

        # 定时器：固定频率发布当前指令
        self._timer_period = 1.0 / rate
        self._timer = self.create_timer(self._timer_period, self._on_timer)

        # 订阅 odom，保存最近一次消息
        self._last_odom = None
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        self.get_logger().info(
            f'MotionSimple started, cmd={cmd_topic}, odom={odom_topic}, '
            f'rate={rate}Hz, vmax={max_linear}, wmax={max_angular}'
        )

    # ================== ODOM 相关 ================== #

    def _odom_callback(self, msg: Odometry):
        self._last_odom = msg

    def has_odom(self) -> bool:
        """是否已经收到过 odom 消息"""
        return self._last_odom is not None

    def get_xy_yaw(self):
        """
        返回 (x, y, yaw)。如果还没有 odom，返回 None。
        """
        if self._last_odom is None:
            return None

        p = self._last_odom.pose.pose.position
        q = self._last_odom.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return p.x, p.y, yaw

    # ================== 速度控制 ================== #

    def _on_timer(self):
        """定时器回调：固定频率发布当前 Twist"""
        self._pub.publish(self._cmd)

    def set_cmd(self, linear_x: float, angular_z: float):
        """
        设置当前速度指令（会限幅）
        """
        v = max(min(linear_x, self.max_linear), -self.max_linear)
        w = max(min(angular_z, self.max_angular), -self.max_angular)

        self._cmd.linear.x = v
        self._cmd.linear.y = 0.0
        self._cmd.linear.z = 0.0
        self._cmd.angular.x = 0.0
        self._cmd.angular.y = 0.0
        self._cmd.angular.z = w

    def stop(self):
        """速度归零"""
        self._cmd = Twist()
        self._pub.publish(self._cmd)
        self.get_logger().info('Stop command sent')
