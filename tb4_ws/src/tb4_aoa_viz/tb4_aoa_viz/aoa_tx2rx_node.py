import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

def yaw_from_quaternion(q):
    # Z轴朝向的偏航角
    ysqr = q.y * q.y
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
    return math.atan2(t3, t4)

def wrap_to_pi(a):
    # 归一化到 [-pi, pi]
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a

class AOATx2RxNode(Node):
    """
    根据 TX 位置与机器人基座位姿，发布相对机器人朝向的 AOA 角 (rad) 到 /aoa_angle:
      aoa = atan2(Tx_y - Rx_y, Tx_x - Rx_x) - base_yaw
    其中 Rx 位姿从 TF(map->base_link)获取；TX 可用常量参数或订阅 /tx_pose 动态更新。
    """
    def __init__(self):
        super().__init__('aoa_tx2rx_node')

        # 参数
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('aoa_topic', '/aoa_angle')
        self.declare_parameter('tx_pose_topic', '/tx_pose')     # geometry_msgs/PoseStamped，可选
        self.declare_parameter('use_tx_topic', False)           # 若 True 则使用订阅的 TX
        self.declare_parameter('tx_x', 0.0)                     # 若不订阅，用固定 TX
        self.declare_parameter('tx_y', 0.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.aoa_topic = self.get_parameter('aoa_topic').value
        self.tx_pose_topic = self.get_parameter('tx_pose_topic').value
        self.use_tx_topic = self.get_parameter('use_tx_topic').value

        # TF
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 发布者
        self.pub = self.create_publisher(Float32, self.aoa_topic, 10)

        # TX 来源
        self.tx_xy: Optional[tuple[float, float]] = None
        if self.use_tx_topic:
            self.create_subscription(PoseStamped, self.tx_pose_topic, self._tx_cb, 10)
            self.get_logger().info(f'Using TX from topic: {self.tx_pose_topic}')
        else:
            x = float(self.get_parameter('tx_x').value)
            y = float(self.get_parameter('tx_y').value)
            self.tx_xy = (x, y)
            self.get_logger().info(f'Using fixed TX: ({x:.3f}, {y:.3f}) in frame "{self.map_frame}"')

        # 定时发布
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1e-3, rate), self._tick)

        self.get_logger().info(
            f'Publishing AOA on {self.aoa_topic}; map="{self.map_frame}", base="{self.base_frame}", rate={rate} Hz'
        )

    def _tx_cb(self, msg: PoseStamped):
        # 假设 TX 提供的是 map 坐标系（如非 map，可加入 tf 转换）
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            try:
                # 转到 map
                t = self._tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, rclpy.time.Time())
                # 这里简单做平移旋转：把点转到 map（仅使用平移+Yaw）
                # 更严谨可用 tf2_geometry_msgs，但此处点到点足够
                # 平移
                tx = t.transform.translation
                # 旋转只考虑 yaw
                yaw = yaw_from_quaternion(t.transform.rotation)
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                px = msg.pose.position.x
                py = msg.pose.position.y
                x_m = tx.x + cos_y * px - sin_y * py
                y_m = tx.y + sin_y * px + cos_y * py
                self.tx_xy = (x_m, y_m)
            except (LookupException, ConnectivityException, ExtrapolationException):
                # 转换失败，保持上一帧
                pass
        else:
            self.tx_xy = (msg.pose.position.x, msg.pose.position.y)

    def _tick(self):
        # 需要 TX 与 RX 位姿
        if self.tx_xy is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        rx_x = tf.transform.translation.x
        rx_y = tf.transform.translation.y
        base_yaw = yaw_from_quaternion(tf.transform.rotation)

        tx_x, tx_y = self.tx_xy
        los_yaw_global = math.atan2(tx_y - rx_y, tx_x - rx_x)  # map 中的 LOS 方向
        aoa_rel = wrap_to_pi(los_yaw_global - base_yaw)        # 相对机体朝向

        self.pub.publish(Float32(data=aoa_rel))

def main():
    rclpy.init()
    node = AOATx2RxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
