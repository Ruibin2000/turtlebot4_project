### Configure the support ros2 publisher to draw the slam marker & aoa publisher bridge



**Standard ros2 package workspace**

```
# 1. Create the workspace if it doesn't exist
mkdir -p ~/tb4_ws/src
cd ~/tb4_ws/src

# 2. Create a new pure-Python package
ros2 pkg create --build-type ament_python tb4_aoa_viz
```



then modify the info to assign python builder

```
nano ~/tb4_ws/src/tb4_aoa_viz/package.xml
```



```
<?xml version="1.0"?>
<package format="3">
  <name>tb4_aoa_viz</name>
  <version>0.0.1</version>
  <description>AOA visualization markers in RViz</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```



```
nano ~/tb4_ws/src/tb4_aoa_viz/setup.py
```



```
from setuptools import setup

package_name = 'tb4_aoa_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['tb4_aoa_viz/aoa_marker.launch.py',
                                   'tb4_aoa_viz/aoa_tx2rx.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='AOA visualization markers in RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aoa_marker_node = tb4_aoa_viz.aoa_marker_node:main',
            'aoa_tx2rx_node = tb4_aoa_viz.aoa_tx2rx_node:main',
        ],
    },
)
```



```
nano ~/tb4_ws/src/tb4_aoa_viz/setup.cfg
```



```
[develop]
script_dir=$base/lib/tb4_aoa_viz
[install]
install_scripts=$base/lib/tb4_aoa_viz
```



**Next, create the ros2 marker array node publisher**

```
nano ~/tb4_ws/src/tb4_aoa_viz/resource/tb4_aoa_viz
```



```
tb4_aoa_viz
```



The drawing program

```
nano ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz/aoa_marker_node.py
```



```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from collections import deque
from typing import Optional, Deque, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AOAMarkerNode(Node):
    """
    AOA + SNR 可视化节点：
      - 角度门限：|true_local_aoa| <= aoa_abs_deg_limit 才绘制 AOA 箭头/轨迹
      - SNR 突降抑制：滑窗中位数 + MAD；抑制期不画 AOA 箭头与采样轨迹
      - 轨迹渐隐：指数衰减（半衰期 fade_half_life_sec），超龄删除
      - 新增：蓝色航向箭头（始终显示）
      - 新增：紫罗兰色 AoA 轨迹，长度随 SNR 线性缩放（SNR_min→最短，SNR_max→最长）
    """

    _last_published_trail_count: int = 0

    def __init__(self):
        super().__init__('aoa_marker_node')

        # 话题/帧/单位
        self.declare_parameter('aoa_topic', '/aoa_angle')
        self.declare_parameter('snr_topic', '/snr_db')
        self.declare_parameter('marker_topic', '/aoa_markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('angle_in_degrees', False)

        # 几何门限 & TX 位置
        self.declare_parameter('tx_x', 0.0)
        self.declare_parameter('tx_y', 0.0)
        self.declare_parameter('aoa_abs_deg_limit', 30.0)

        # AOA 箭头样式
        self.declare_parameter('arrow_length', 1.0)
        self.declare_parameter('shaft_diameter', 0.03)
        self.declare_parameter('head_diameter', 0.08)
        self.declare_parameter('head_length', 0.15)

        # 航向（机器人 orientation）箭头
        self.declare_parameter('orient_arrow_enable', True)
        self.declare_parameter('orient_arrow_length', 1.0)
        self.declare_parameter('orient_shaft_diameter', 0.03)
        self.declare_parameter('orient_head_diameter', 0.08)
        self.declare_parameter('orient_head_length', 0.15)

        # 轨迹采样与样式（按真实时间衰减）
        self.declare_parameter('trail_period', 6.0)
        # 轨迹长度（SNR 映射）
        self.declare_parameter('trail_length_min', 3.0)   # SNR_min 对应的最短长度
        self.declare_parameter('trail_length', 10.0)      # 作为最大长度（向后兼容）
        self.declare_parameter('trail_line_width', 0.02)
        self.declare_parameter('trail_alpha_max', 0.90)
        self.declare_parameter('trail_fade_half_life_sec', 20.0)
        self.declare_parameter('trail_max_age_sec', 60.0)
        self.declare_parameter('trail_capacity', 200)

        # 轨迹颜色（紫罗兰色）
        self._trail_rgb = (0.56, 0.0, 1.0)  # violet

        # SNR → 轨迹长度线性映射参数
        self.declare_parameter('snr_min_db', 0.0)    # 对应最短
        self.declare_parameter('snr_max_db', 20.0)   # 对应最长

        # SNR 异常突降检测
        self.declare_parameter('snr_outlier_enable', True)
        self.declare_parameter('snr_window_sec', 5.0)
        self.declare_parameter('snr_min_samples', 5)
        self.declare_parameter('snr_drop_db', 3.0)
        self.declare_parameter('snr_k_mad', 2.5)
        self.declare_parameter('snr_rel_drop_frac', 0.30)
        self.declare_parameter('snr_hold_sec', 2.0)

        # ROS I/O
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._aoa_sub = self.create_subscription(
            Float32, self.get_parameter('aoa_topic').value, self._aoa_cb, 10
        )
        self._snr_sub = self.create_subscription(
            Float32, self.get_parameter('snr_topic').value, self._snr_cb, 10
        )

        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10
        )
        self._snr_outlier_pub = self.create_publisher(Bool, '/snr_outlier', 10)
        self._snr_median_pub  = self.create_publisher(Float32, '/snr_median', 10)

        # 运行态
        self._last_angle: Optional[float] = None  # rad
        self._last_snr_db: Optional[float] = None
        self._last_tf_ok = False

        # SNR 历史 & 抑制计时
        self._snr_hist: Deque[Tuple[float, float]] = deque()
        self._snr_outlier_until: float = 0.0

        # 轨迹：[(p0, p1, t_birth_sec)]
        self._trails: List[Tuple[Point, Point, float]] = []

        # 定时器
        self._timer = self.create_timer(0.1, self._tick_arrow)
        self._trail_timer = self.create_timer(
            float(self.get_parameter('trail_period').value), self._sample_trail
        )

        # 动态参数
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"AOA:{self.get_parameter('aoa_topic').value}  "
            f"SNR:{self.get_parameter('snr_topic').value}  "
            f"Markers->{self.get_parameter('marker_topic').value}  "
            f"TX=({self.get_parameter('tx_x').value}, {self.get_parameter('tx_y').value})  "
            f"AOA limit={self.get_parameter('aoa_abs_deg_limit').value} deg"
        )

    # ---------------- 参数回调 ----------------
    def _on_set_params(self, params) -> SetParametersResult:
        try:
            for p in params:
                if p.name == 'trail_period':
                    period = max(0.1, float(p.value))
                    try:
                        self._trail_timer.cancel()
                    except Exception:
                        pass
                    self._trail_timer = self.create_timer(period, self._sample_trail)
                elif p.name == 'trail_capacity':
                    cap = int(p.value)
                    if cap <= 0:
                        cap = 1
                    if len(self._trails) > cap:
                        self._trails = self._trails[-cap:]
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    # ---------------- 订阅回调 ----------------
    def _aoa_cb(self, msg: Float32):
        if self.get_parameter('angle_in_degrees').value:
            self._last_angle = math.radians(float(msg.data))
        else:
            self._last_angle = float(msg.data)

    def _snr_cb(self, msg: Float32):
        self._last_snr_db = float(msg.data)
        t_now = self._now_sec()
        self._snr_hist.append((t_now, self._last_snr_db))

        win = float(self.get_parameter('snr_window_sec').value)
        while self._snr_hist and (t_now - self._snr_hist[0][0] > win):
            self._snr_hist.popleft()

        self._update_snr_outlier_flag(t_now)

    # ---------------- 工具函数 ----------------
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _yaw_from_quaternion(self, q) -> float:
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        return math.atan2(t3, t4)

    def _lookup_base_in(self, map_frame: str, base_frame: str):
        try:
            if not self._tf_buffer.can_transform(
                map_frame, base_frame, rclpy.time.Time(), timeout=RclpyDuration(seconds=0.2)
            ):
                if self._last_tf_ok:
                    self.get_logger().warn('TF not available: map -> %s' % base_frame)
                self._last_tf_ok = False
                return None
            tf = self._tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(), timeout=RclpyDuration(seconds=0.2)
            )
            self._last_tf_ok = True
            return tf
        except Exception as e:
            if self._last_tf_ok:
                self.get_logger().warn(f'lookup_transform({map_frame}->{base_frame}) failed: {e}')
            self._last_tf_ok = False
            return None

    def _snr_drop_blocked(self) -> bool:
        if not self.get_parameter('snr_outlier_enable').value:
            return False
        return self._now_sec() < self._snr_outlier_until

    def _true_local_aoa_ok(self, tf) -> bool:
        tx_x = float(self.get_parameter('tx_x').value)
        tx_y = float(self.get_parameter('tx_y').value)

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = self._yaw_from_quaternion(tf.transform.rotation)

        dx = tx_x - rx
        dy = tx_y - ry
        if dx == 0.0 and dy == 0.0:
            return True

        bearing_map = math.atan2(dy, dx)
        true_local_aoa = wrap_to_pi(bearing_map - yaw)

        limit_rad = math.radians(float(self.get_parameter('aoa_abs_deg_limit').value))
        return abs(true_local_aoa) <= limit_rad

    def _delete_arrow_only(self):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.get_parameter('map_frame').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aoa'
        m.id = 0
        m.action = Marker.DELETE
        arr.markers.append(m)
        self._marker_pub.publish(arr)

    # ---------------- 定时刷新箭头们 ----------------
    def _tick_arrow(self):
        """
        周期刷新：
          - AOA 箭头：受 SNR 抑制 & 真-AOA 门限
          - 航向箭头：始终尝试显示（不受抑制）
          - 刷新/清理 AoA 轨迹
        """
        arr = MarkerArray()

        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        tf = self._lookup_base_in(map_frame, base_frame)

        # --- 航向（蓝色）箭头 ---
        if self.get_parameter('orient_arrow_enable').value and tf is not None:
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = self._yaw_from_quaternion(tf.transform.rotation)
            L = float(self.get_parameter('orient_arrow_length').value)

            p0 = Point(x=x, y=y, z=0.12)
            p1 = Point(x=x + L * math.cos(yaw), y=y + L * math.sin(yaw), z=0.12)

            arrow = Marker()
            arrow.header.frame_id = map_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'heading'
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [p0, p1]
            arrow.scale.x = float(self.get_parameter('orient_shaft_diameter').value)
            arrow.scale.y = float(self.get_parameter('orient_head_diameter').value)
            arrow.scale.z = float(self.get_parameter('orient_head_length').value)
            # 蓝色
            arrow.color.r = 0.0
            arrow.color.g = 0.4
            arrow.color.b = 1.0
            arrow.color.a = 1.0
            arrow.lifetime = Duration(sec=0, nanosec=0)
            arr.markers.append(arrow)
        else:
            # 删除航向箭头
            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'heading'
            m.id = 1
            m.action = Marker.DELETE
            arr.markers.append(m)

        # --- AOA（橙红色）箭头 ---
        blocked = self._snr_drop_blocked()
        if not blocked and self._last_angle is not None and tf is not None and self._true_local_aoa_ok(tf):
            L = float(self.get_parameter('arrow_length').value)
            shaft_d = float(self.get_parameter('shaft_diameter').value)
            head_d = float(self.get_parameter('head_diameter').value)
            head_L = float(self.get_parameter('head_length').value)

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            base_yaw = self._yaw_from_quaternion(tf.transform.rotation)
            theta = base_yaw + self._last_angle

            p0 = Point(x=x, y=y, z=0.1)
            p1 = Point(x=x + L * math.cos(theta), y=y + L * math.sin(theta), z=0.1)

            arrow = Marker()
            arrow.header.frame_id = map_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'aoa'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [p0, p1]
            arrow.scale.x = shaft_d
            arrow.scale.y = head_d
            arrow.scale.z = head_L
            arrow.color.r = 1.0
            arrow.color.g = 0.2
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            arrow.lifetime = Duration(sec=0, nanosec=0)
            arr.markers.append(arrow)
        else:
            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa'
            m.id = 0
            m.action = Marker.DELETE
            arr.markers.append(m)

        # 刷新/构建 AoA 轨迹（紫罗兰 & 渐隐）
        arr.markers.extend(self._build_time_fading_trails())
        self._marker_pub.publish(arr)

    # ---------------- 轨迹采样 ----------------
    def _sample_trail(self):
        """按 trail_period 采样 AoA 线段，长度按 SNR 线性缩放，且受抑制与角度门限。"""
        if self._snr_drop_blocked() or self._last_angle is None:
            return

        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        tf = self._lookup_base_in(map_frame, base_frame)
        if tf is None or not self._true_local_aoa_ok(tf):
            return

        # --- SNR → 长度线性映射 ---
        snr = self._last_snr_db
        snr_min = float(self.get_parameter('snr_min_db').value)
        snr_max = float(self.get_parameter('snr_max_db').value)
        L_min = float(self.get_parameter('trail_length_min').value)
        L_max = float(self.get_parameter('trail_length').value)  # 向后兼容：作为最大长度
        if L_max < L_min:
            L_max, L_min = L_min, L_max  # 容错：交换

        if snr is None or snr_max <= snr_min:
            LL = L_min
        else:
            t = (max(snr_min, min(snr, snr_max)) - snr_min) / (snr_max - snr_min)
            LL = L_min + t * (L_max - L_min)

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        base_yaw = self._yaw_from_quaternion(tf.transform.rotation)
        theta = base_yaw + self._last_angle

        p0 = Point(x=x, y=y, z=0.02)
        p1 = Point(x=x + LL * math.cos(theta), y=y + LL * math.sin(theta), z=0.02)

        # 容量控制
        cap = int(self.get_parameter('trail_capacity').value)
        if cap <= 0:
            cap = 1
        if len(self._trails) >= cap:
            self._trails = self._trails[-(cap - 1):]

        self._trails.append((p0, p1, self._now_sec()))

    # ---------------- 按真实时间渐隐的轨迹构建 ----------------
    def _build_time_fading_trails(self) -> List[Marker]:
        trail_width = float(self.get_parameter('trail_line_width').value)
        alpha_max = float(self.get_parameter('trail_alpha_max').value)
        half_life = max(1e-3, float(self.get_parameter('trail_fade_half_life_sec').value))
        max_age = max(half_life, float(self.get_parameter('trail_max_age_sec').value))

        now = self._now_sec()
        survivors: List[Tuple[Point, Point, float]] = []
        markers: List[Marker] = []

        r_v, g_v, b_v = self._trail_rgb

        for idx, (p0, p1, t_birth) in enumerate(self._trails):
            age = now - t_birth
            if age > max_age:
                continue
            alpha = alpha_max * (0.5 ** (age / half_life))
            if alpha < 0.02:
                continue

            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa_trails'
            m.id = 100 + idx
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.points = [p0, p1]
            m.scale.x = trail_width
            # 紫罗兰色
            m.color.r = float(r_v)
            m.color.g = float(g_v)
            m.color.b = float(b_v)
            m.color.a = float(alpha)
            m.lifetime = Duration(sec=0, nanosec=0)
            markers.append(m)
            survivors.append((p0, p1, t_birth))

        self._trails = survivors

        if len(self._trails) < self._last_published_trail_count:
            for kill in range(len(self._trails), self._last_published_trail_count):
                m = Marker()
                m.header.frame_id = self.get_parameter('map_frame').value
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'aoa_trails'
                m.id = 100 + kill
                m.action = Marker.DELETE
                markers.append(m)
        self._last_published_trail_count = len(self._trails)
        return markers

    # ---------------- SNR 异常突降检测 ----------------
    def _update_snr_outlier_flag(self, t_now: float):
        if not self.get_parameter('snr_outlier_enable').value:
            self._snr_outlier_pub.publish(Bool(data=False))
            return

        min_n = int(self.get_parameter('snr_min_samples').value)
        if len(self._snr_hist) < max(2, min_n):
            self._snr_outlier_pub.publish(Bool(data=False))
            return

        vals = [v for (_, v) in self._snr_hist]
        vals_sorted = sorted(vals)
        n = len(vals_sorted)
        med = vals_sorted[n // 2] if n % 2 == 1 else 0.5 * (vals_sorted[n//2 - 1] + vals_sorted[n//2])

        abs_dev = [abs(v - med) for v in vals_sorted]
        abs_dev_sorted = sorted(abs_dev)
        mad = abs_dev_sorted[n // 2] if n % 2 == 1 else 0.5 * (abs_dev_sorted[n//2 - 1] + abs_dev_sorted[n//2])
        sigma_robust = 1.4826 * mad

        snr_now = vals[-1]
        drop_db = float(self.get_parameter('snr_drop_db').value)
        k_mad = float(self.get_parameter('snr_k_mad').value)
        rel_frac = float(self.get_parameter('snr_rel_drop_frac').value)

        cond_abs = (med - snr_now) >= drop_db
        cond_rel = (med > 1e-6) and ((med - snr_now) / max(1e-6, med) >= rel_frac)
        cond_mad = (sigma_robust > 0.0) and ((med - snr_now) >= k_mad * sigma_robust)

        is_outlier = bool(cond_abs or cond_rel or cond_mad)
        if is_outlier:
            hold = float(self.get_parameter('snr_hold_sec').value)
            self._snr_outlier_until = max(self._snr_outlier_until, t_now + hold)
            self.get_logger().debug(
                f"SNR outlier: now={snr_now:.2f} dB, med={med:.2f} dB, "
                f"drop={med - snr_now:.2f} dB, sigma~={sigma_robust:.2f}, hold={hold}s"
            )

        self._snr_outlier_pub.publish(Bool(data=is_outlier or (t_now < self._snr_outlier_until)))
        self._snr_median_pub.publish(Float32(data=float(med)))

    # ---------------- 生命周期 ----------------
    def on_shutdown(self):
        arr = MarkerArray()

        # 删除 AOA 箭头
        m0 = Marker()
        m0.header.frame_id = self.get_parameter('map_frame').value
        m0.header.stamp = self.get_clock().now().to_msg()
        m0.ns = 'aoa'
        m0.id = 0
        m0.action = Marker.DELETE
        arr.markers.append(m0)

        # 删除 航向箭头
        mh = Marker()
        mh.header.frame_id = self.get_parameter('map_frame').value
        mh.header.stamp = self.get_clock().now().to_msg()
        mh.ns = 'heading'
        mh.id = 1
        mh.action = Marker.DELETE
        arr.markers.append(mh)

        # 删除 trails
        for idx in range(self._last_published_trail_count):
            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa_trails'
            m.id = 100 + idx
            m.action = Marker.DELETE
            arr.markers.append(m)

        self._marker_pub.publish(arr)


def main():
    rclpy.init()
    node = AOAMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.on_shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```



launch file

```
nano ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz/aoa_marker.launch.py
```



```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb4_aoa_viz',
            executable='aoa_marker_node',
            name='aoa_marker_node',
            output='screen',
            parameters=[{
                # --- 话题设置 ---
                'aoa_topic': '/aoa_angle',
                'snr_topic': '/snr_db',
                'marker_topic': '/aoa_markers',

                # --- TF / 几何门限 & TX 位置 ---
                'map_frame': 'map',
                'base_frame': 'base_link',
                'tx_x': 1.75991,
                'tx_y': 0.602680,
                'aoa_abs_deg_limit': 25.0,
                'angle_in_degrees': False,

                # --- AOA 箭头外观 ---
                'arrow_length': 1.0,
                'shaft_diameter': 0.03,
                'head_diameter': 0.08,
                'head_length': 0.15,

                # --- 航向（蓝色）箭头 ---
                'orient_arrow_enable': True,
                'orient_arrow_length': 1.2,
                'orient_shaft_diameter': 0.03,
                'orient_head_diameter': 0.08,
                'orient_head_length': 0.15,

                # --- 轨迹采样与渐隐（紫罗兰色） ---
                'trail_period': 6.0,               # 每 6 秒采样一条 AoA 线段
                'trail_length_min': 3.0,            # SNR_min → 3 m
                'trail_length': 10.0,               # SNR_max → 10 m（最大长度）
                'trail_line_width': 0.02,
                'trail_alpha_max': 0.9,
                'trail_fade_half_life_sec': 20.0,
                'trail_max_age_sec': 60.0,
                'trail_capacity': 200,

                # --- SNR → 长度映射 ---
                'snr_min_db': 0.0,                  # 最短线对应的 SNR
                'snr_max_db': 20.0,                 # 最长线对应的 SNR

                # --- SNR 异常检测（可按需开启/关闭） ---
                'snr_outlier_enable': True,
                'snr_window_sec': 5.0,
                'snr_min_samples': 5,
                'snr_drop_db': 15.0,
                'snr_k_mad': 2.5,
                'snr_rel_drop_frac': 0.6,
                'snr_hold_sec': 0.5,
            }]
        ),
    ])
```



**Next, create the ros2 publishers**, replace it with the antenna aoa receiver program

provide the true aoa publisher interface

```
nano ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz/aoa_bridge.py
```



```
#!/usr/bin/env python3
# Lightweight ROS 2 publishing bridge for AOA (Float32, radians).

import threading
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32


class _AOANode(Node):
    """Internal node that owns the /aoa_angle publisher."""
    def __init__(self, topic: str = "/aoa_angle", queue_size: int = 10):
        super().__init__("aoa_bridge_node")
        self._pub = self.create_publisher(Float32, topic, queue_size)

    def publish_angle(self, angle_rad: float) -> None:
        msg = Float32()
        msg.data = float(angle_rad)
        self._pub.publish(msg)


class AOAPublisher:
    """Callee: expose publish(angle_rad); manages rclpy + executor thread."""
    def __init__(self, topic: str = "/aoa_angle"):
        self._topic = topic
        self._node: Optional[_AOANode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = _AOANode(topic=self._topic)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin():
            try:
                self._executor.spin()
            except Exception:
                pass

        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()
        self._started = True

    def publish(self, angle_rad: float) -> None:
        if not self._started:
            self.start()
        assert self._node is not None
        self._node.publish_angle(angle_rad)

    def stop(self) -> None:
        if not self._started:
            return
        assert self._executor is not None and self._node is not None
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._node = None
        self._executor = None
        self._thread = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()


# Convenience: return a callable publish_aoa(angle_rad)
class _PublishAOAFnHolder:
    _lock = threading.Lock()
    _publisher: Optional[AOAPublisher] = None

    @classmethod
    def get_publish_aoa_fn(cls, topic: str = "/aoa_angle") -> Callable[[float], None]:
        with cls._lock:
            if cls._publisher is None:
                cls._publisher = AOAPublisher(topic=topic)
                cls._publisher.start()

        def publish_aoa(angle_rad: float) -> None:
            assert cls._publisher is not None
            cls._publisher.publish(angle_rad)

        return publish_aoa


def get_publish_aoa_fn(topic: str = "/aoa_angle") -> Callable[[float], None]:
    """Usage: publish_aoa = get_publish_aoa_fn(); publish_aoa(angle_rad)"""
    return _PublishAOAFnHolder.get_publish_aoa_fn(topic=topic)
```



provide the true snr publisher interface

```
nano ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz/snr_bridge.py
```



```
#!/usr/bin/env python3
# Lightweight ROS 2 publishing bridge for SNR (Float32, dB).

import threading
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32


class _SNRNode(Node):
    """Internal node that owns the /snr_db publisher."""
    def __init__(self, topic: str = "/snr_db", queue_size: int = 10):
        super().__init__("snr_bridge_node")
        self._pub = self.create_publisher(Float32, topic, queue_size)

    def publish_snr_db(self, snr_db: float) -> None:
        msg = Float32()
        msg.data = float(snr_db)
        self._pub.publish(msg)


class SNRPublisher:
    """Callee: expose publish(snr_db); manages rclpy + executor thread."""
    def __init__(self, topic: str = "/snr_db"):
        self._topic = topic
        self._node: Optional[_SNRNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = _SNRNode(topic=self._topic)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin():
            try:
                self._executor.spin()
            except Exception:
                pass

        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()
        self._started = True

    def publish(self, snr_db: float) -> None:
        if not self._started:
            self.start()
        assert self._node is not None
        self._node.publish_snr_db(snr_db)

    def stop(self) -> None:
        if not self._started:
            return
        assert self._executor is not None and self._node is not None
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._node = None
        self._executor = None
        self._thread = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()


# Convenience: return a callable publish_snr(snr_db)
class _PublishSNRFnHolder:
    _lock = threading.Lock()
    _publisher: Optional[SNRPublisher] = None

    @classmethod
    def get_publish_snr_fn(cls, topic: str = "/snr_db") -> Callable[[float], None]:
        with cls._lock:
            if cls._publisher is None:
                cls._publisher = SNRPublisher(topic=topic)
                cls._publisher.start()

        def publish_snr(snr_db: float) -> None:
            assert cls._publisher is not None
            cls._publisher.publish(snr_db)

        return publish_snr


def get_publish_snr_fn(topic: str = "/snr_db") -> Callable[[float], None]:
    """Usage: publish_snr = get_publish_snr_fn(); publish_snr(snr_db)"""
    return _PublishSNRFnHolder.get_publish_snr_fn(topic=topic)
```



### Motion Control

```
cd ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz
nano motion_bridge_map.py
```



```
#!/usr/bin/env python3
# Motion bridge (using map frame) for TurtleBot4.
#
# 对外接口：
#   move_bot = get_move_bot_fn()
#   move_bot(x_target, y_target, yaw_target)
#
# 内部使用 TF 查 map->base_frame 位姿，用 map 坐标系做闭环纠偏。

import math
import threading
import time
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


# ================= 工具函数 ================= #

def normalize_angle(angle: float) -> float:
    """归一化到 (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从 quaternion 提取 yaw (绕 z 轴)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ================= 内部 Node ================= #

class _MotionNode(Node):
    """
    - 发布 /cmd_vel
    - 通过 TF 获取 map->base_frame 的变换 (pose in map)
    """

    def __init__(
        self,
        cmd_vel_topic: str = "/cmd_vel",
        global_frame: str = "map",
        base_frame: str = "base_link",   # TB4 常见：base_link 或 base_footprint
        queue_size: int = 10,
    ):
        super().__init__("motion_bridge_map_node")

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, queue_size)

        self._global_frame = global_frame
        self._base_frame = base_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._have_tf = threading.Event()
        self.create_timer(0.1, self._check_tf_once)

    def _check_tf_once(self):
        if self._have_tf.is_set():
            return
        pose = self.get_pose()
        if pose is not None:
            self._have_tf.set()

    def wait_for_tf(self, timeout: float = 5.0) -> bool:
        return self._have_tf.wait(timeout=timeout)

    # --- 关键：通过 TF 拿 map 下的位姿 --- #

    def get_pose(self) -> Optional[Tuple[float, float, float, Tuple[float, float, float, float]]]:
        """
        返回 (x, y, yaw, (qx, qy, qz, qw)) in map frame。
        没拿到 TF 就返回 None。
        """
        try:
            now = rclpy.time.Time()
            t = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException:
            return None

        trans = t.transform.translation
        rot = t.transform.rotation

        x = float(trans.x)
        y = float(trans.y)
        qx = float(rot.x)
        qy = float(rot.y)
        qz = float(rot.z)
        qw = float(rot.w)
        yaw = quat_to_yaw(qx, qy, qz, qw)

        return x, y, yaw, (qx, qy, qz, qw)

    # --- 速度控制 --- #

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_cmd(0.0, 0.0)


# ================= MotionController ================= #

class MotionController:
    """
    使用 map 坐标系进行闭环纠偏的控制器。

    外部调用：
        ctrl = MotionController()
        ctrl.start()
        ctrl.move_to(x_target, y_target, yaw_target)
        ctrl.stop()
    """

    def __init__(
        self,
        cmd_vel_topic: str = "/cmd_vel",
        global_frame: str = "map",
        base_frame: str = "base_link",
    ):
        self._cmd_vel_topic = cmd_vel_topic
        self._global_frame = global_frame
        self._base_frame = base_frame

        self._node: Optional[_MotionNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._started = False

        # 控制参数，可根据 TB4 实机调
        self.max_ang_vel = 0.6
        self.max_lin_vel = 0.22
        self.kp_ang = 1.5
        self.kp_lin = 0.8

        self.yaw_tol = math.radians(2.0)   # 角度误差容忍
        self.dist_tol = 0.02               # 距离误差容忍（米）

    # -------- 生命周期管理 -------- #

    def start(self) -> None:
        if self._started:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = _MotionNode(
            cmd_vel_topic=self._cmd_vel_topic,
            global_frame=self._global_frame,
            base_frame=self._base_frame,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin():
            try:
                self._executor.spin()
            except Exception:
                pass

        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()
        self._started = True

        if not self._node.wait_for_tf(timeout=5.0):
            self._node.get_logger().warn(
                f"TF {self._global_frame} -> {self._base_frame} not available yet."
            )

    def stop(self) -> None:
        if not self._started:
            return

        assert self._node is not None
        assert self._executor is not None

        self._node.stop_robot()
        time.sleep(0.1)

        self._executor.remove_node(self._node)
        self._executor.shutdown()

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        self._node = None
        self._executor = None
        self._thread = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    # -------- 外部接口：移动到 map 中的目标 pose -------- #

    def move_to(self, x_target: float, y_target: float, yaw_target: float) -> None:
        """
        输入：目标 pose in map
            x_target, y_target (米)
            yaw_target (弧度)

        步骤：
          1. 在 map 坐标系下，从当前 (x,y,yaw) 走到 (x_target, y_target)
          2. 到点后，再旋转到 yaw_target
        """
        if not self._started:
            self.start()

        assert self._node is not None

        pose0 = self._node.get_pose()
        if pose0 is None:
            self._node.get_logger().warn("No map pose available; abort move.")
            return

        x0, y0, yaw0, quat0 = pose0
        print(
            f"[MotionController] Start pose in map: "
            f"x={x0:.3f}, y={y0:.3f}, yaw={yaw0:.3f} rad"
        )

        # 1) 走到目标 (x_target, y_target)
        self._move_to_xy(x_target, y_target)

        # 2) 旋转到目标 yaw_target
        self._rotate_to_yaw(yaw_target)

        # 最终姿态打印
        posef = self._node.get_pose()
        if posef is None:
            print("[MotionController] Final map pose unavailable.")
            return

        xf, yf, yawf, quatf = posef
        print(
            "[MotionController] Final pose in map: "
            f"x={xf:.3f} m, y={yf:.3f} m, yaw={yawf:.3f} rad "
            f"({math.degrees(yawf):.1f} deg)"
        )
        print(
            "[MotionController] Final orientation (qx, qy, qz, qw) = "
            f"({quatf[0]:.3f}, {quatf[1]:.3f}, {quatf[2]:.3f}, {quatf[3]:.3f})"
        )

    # -------- 低层控制：yaw + xy -------- #

    def _rotate_to_yaw(self, target_yaw: float, timeout: float = 15.0) -> None:
        assert self._node is not None
        t_start = time.time()

        target_yaw = normalize_angle(target_yaw)

        while rclpy.ok() and (time.time() - t_start) < timeout:
            pose = self._node.get_pose()
            if pose is None:
                time.sleep(0.01)
                continue

            _, _, yaw, _ = pose
            err = normalize_angle(target_yaw - yaw)
            if abs(err) < self.yaw_tol:
                break

            w = self.kp_ang * err
            w = max(-self.max_ang_vel, min(self.max_ang_vel, w))
            self._node.publish_cmd(0.0, w)
            time.sleep(0.02)

        self._node.stop_robot()
        time.sleep(0.05)

    def _move_to_xy(self, target_x: float, target_y: float, timeout: float = 30.0) -> None:
        """
        在 map 帧下走到 (target_x, target_y)。
        用“朝向目标点”的角度做 heading 修正。
        """
        assert self._node is not None

        t_start = time.time()

        while rclpy.ok() and (time.time() - t_start) < timeout:
            pose = self._node.get_pose()
            if pose is None:
                time.sleep(0.01)
                continue

            x, y, yaw, _ = pose
            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.dist_tol:
                break

            desired_yaw = math.atan2(dy, dx)
            heading_err = normalize_angle(desired_yaw - yaw)

            # 线速度：距离越远越快，上限 max_lin_vel，下限 0.05
            v = self.kp_lin * dist
            v = min(self.max_lin_vel, max(0.05, v))

            # 角速度：纠一点 heading，限制不要太大
            w = self.kp_ang * heading_err
            w = max(-0.4, min(0.4, w))

            self._node.publish_cmd(v, w)
            time.sleep(0.02)

        self._node.stop_robot()
        time.sleep(0.05)


# ================= Convenience：函数接口 ================= #

class _MoveBotFnHolder:
    _lock = threading.Lock()
    _controller: Optional[MotionController] = None

    @classmethod
    def get_move_bot_fn(
        cls,
        cmd_vel_topic: str = "/cmd_vel",
        global_frame: str = "map",
        base_frame: str = "base_link",
    ) -> Callable[[float, float, float], None]:
        """
        返回函数：
            move_bot(x_target, y_target, yaw_target)
        目标 pose 全在 map 坐标系下。
        """
        with cls._lock:
            if cls._controller is None:
                cls._controller = MotionController(
                    cmd_vel_topic=cmd_vel_topic,
                    global_frame=global_frame,
                    base_frame=base_frame,
                )
                cls._controller.start()

        def move_bot(x_target: float, y_target: float, yaw_target: float) -> None:
            assert cls._controller is not None
            cls._controller.move_to(x_target, y_target, yaw_target)

        return move_bot


def get_move_bot_fn(
    cmd_vel_topic: str = "/cmd_vel",
    global_frame: str = "map",
    base_frame: str = "base_link",
) -> Callable[[float, float, float], None]:
    """
    Usage:
        from tb4_aoa_viz.motion_bridge_map import get_move_bot_fn
        move_bot = get_move_bot_fn(base_frame="base_footprint")  # 如使用 base_footprint

        import math
        # 移动到 map 下 (1.0, 0.5)，朝向 pi/2
        move_bot(1.0, 0.5, math.pi / 2)
    """
    return _MoveBotFnHolder.get_move_bot_fn(
        cmd_vel_topic=cmd_vel_topic,
        global_frame=global_frame,
        base_frame=base_frame,
    )

```



### build

```
cd ~/tb4_ws

# 删除上次编译生成的所有文件夹
rm -rf build/ install/ log/

# 重新编译全部包
colcon build

```

