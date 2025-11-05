# Turtlebot4_project

This is the set up file backup repo for turtlebot4 project, including two parts

1. Configure the support ros2 publisher to draw the slam marker & aoa publisher
2. Commends to bringup the visualization



### Configure the support ros2 publisher to draw the slam marker & aoa publisher

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
    """Wrap angle to [-pi, pi].."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AOAMarkerNode(Node):
    """
    AOA + SNR 可视化节点（无固定 SNR 门限；有异常突降屏蔽；轨迹按真实时间衰减）。

    - 角度门限：仅当“真本地AOA” |aoa| <= aoa_abs_deg_limit 才绘制箭头/轨迹
    - SNR 突降（基于滑窗中位数+MAD）：在 snr_hold_sec 抑制期内屏蔽绘制
    - 轨迹渐隐：每条线段记录时间戳，按指数衰减（半衰期 fade_half_life_sec），到期移除
    """

    _last_published_trail_count: int = 0  # 兼容删除旧ID（可留作保险）

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

        # 箭头样式
        self.declare_parameter('arrow_length', 1.0)
        self.declare_parameter('shaft_diameter', 0.03)
        self.declare_parameter('head_diameter', 0.08)
        self.declare_parameter('head_length', 0.15)

        # 轨迹采样与样式（不再用“容量内新旧渐隐”，而是按真实时间衰减）
        self.declare_parameter('trail_period', 6.0)          # 采样间隔（秒）
        self.declare_parameter('trail_length', 10.0)         # 每条线段长度（米）
        self.declare_parameter('trail_line_width', 0.02)
        self.declare_parameter('trail_alpha_max', 0.90)      # 新线段初始不透明度
        self.declare_parameter('trail_fade_half_life_sec', 20.0)  # 半衰期，越小衰减越快
        self.declare_parameter('trail_max_age_sec', 60.0)    # 超龄自动删除
        self.declare_parameter('trail_capacity', 200)        # 上限，防止无限增长

        # SNR 异常突降检测（滑窗 + 中位数 + MAD）
        self.declare_parameter('snr_outlier_enable', True)
        self.declare_parameter('snr_window_sec', 5.0)
        self.declare_parameter('snr_min_samples', 5)
        self.declare_parameter('snr_drop_db', 3.0)           # 绝对跌落阈值
        self.declare_parameter('snr_k_mad', 2.5)             # MAD 倍数阈值
        self.declare_parameter('snr_rel_drop_frac', 0.30)    # 相对跌落比例阈值
        self.declare_parameter('snr_hold_sec', 2.0)          # 判定异常后的抑制时间

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
        # 诊断输出（可在 RViz/PlotJuggler 观察）
        self._snr_outlier_pub = self.create_publisher(Bool, '/snr_outlier', 10)
        self._snr_median_pub  = self.create_publisher(Float32, '/snr_median', 10)

        # 运行态
        self._last_angle: Optional[float] = None  # 弧度
        self._last_snr_db: Optional[float] = None
        self._last_tf_ok = False

        # SNR 历史 & 抑制计时
        self._snr_hist: Deque[Tuple[float, float]] = deque()  # (t_sec, snr_db)
        self._snr_outlier_until: float = 0.0

        # 轨迹：[(p0, p1, t_birth_sec)]
        self._trails: List[Tuple[Point, Point, float]] = []

        # 定时器：箭头刷新 + 轨迹采样
        self._timer = self.create_timer(1.0, self._tick_arrow)
        self._trail_timer = self.create_timer(
            float(self.get_parameter('trail_period').value), self._sample_trail
        )

        # 动态参数回调（支持运行期更新）
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
                    # 若当前超出新上限，裁掉最旧的
                    if len(self._trails) > cap:
                        self._trails = self._trails[-cap:]
                # 其余参数实时读取，无需额外处理
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    # ---------------- 订阅回调 ----------------
    def _aoa_cb(self, msg: Float32):
        """接收 AOA。按参数决定是否把度转为弧度。"""
        if self.get_parameter('angle_in_degrees').value:
            self._last_angle = math.radians(float(msg.data))
        else:
            self._last_angle = float(msg.data)

    def _snr_cb(self, msg: Float32):
        """接收 SNR(dB)，记录到滑动窗口并做异常检测。"""
        self._last_snr_db = float(msg.data)
        t_now = self._now_sec()
        self._snr_hist.append((t_now, self._last_snr_db))

        # 修剪窗口
        win = float(self.get_parameter('snr_window_sec').value)
        while self._snr_hist and (t_now - self._snr_hist[0][0] > win):
            self._snr_hist.popleft()

        # 更新 outlier 标志
        self._update_snr_outlier_flag(t_now)

    # ---------------- 工具函数 ----------------
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _yaw_from_quaternion(self, q) -> float:
        """Quaternion -> yaw(rad)."""
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        return math.atan2(t3, t4)

    def _lookup_base_in(self, map_frame: str, base_frame: str):
        """尝试查 map->base_frame TF（带超时预检查）。"""
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
        """是否处于 SNR 异常突降的抑制期."""
        if not self.get_parameter('snr_outlier_enable').value:
            return False
        return self._now_sec() < self._snr_outlier_until

    def _true_local_aoa_ok(self, tf) -> bool:
        """
        几何“真-AOA”门限：
        - map中机器人到TX的方位角 bearing_map
        - 减去机器人 yaw 得到 true_local_aoa（本地）
        - 与 aoa_abs_deg_limit 比较
        """
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
        """仅删除箭头（轨迹靠刷新/到期删除）。"""
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.get_parameter('map_frame').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aoa'
        m.id = 0
        m.action = Marker.DELETE
        arr.markers.append(m)
        self._marker_pub.publish(arr)

    # ---------------- 定时刷新箭头 ----------------
    def _tick_arrow(self):
        """每 ~1s 更新箭头，同时刷新轨迹透明度并删除过期线段。"""
        # 处于 SNR outlier 抑制期：不画箭头，但仍刷新/清理轨迹
        blocked = self._snr_drop_blocked()

        arr = MarkerArray()

        if not blocked and self._last_angle is not None:
            map_frame = self.get_parameter('map_frame').value
            base_frame = self.get_parameter('base_frame').value

            tf = self._lookup_base_in(map_frame, base_frame)
            if tf is not None and self._true_local_aoa_ok(tf):
                # 绘制箭头
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
                # 条件不满足：删除箭头
                m = Marker()
                m.header.frame_id = self.get_parameter('map_frame').value
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'aoa'
                m.id = 0
                m.action = Marker.DELETE
                arr.markers.append(m)
        else:
            # 被 outlier 抑制：删除箭头
            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa'
            m.id = 0
            m.action = Marker.DELETE
            arr.markers.append(m)

        # 刷新/构建轨迹（按真实时间衰减 & 到期删除）
        arr.markers.extend(self._build_time_fading_trails())
        self._marker_pub.publish(arr)

    # ---------------- 轨迹采样 ----------------
    def _sample_trail(self):
        """按 trail_period 采样一条线段，加入轨迹队列（若未被 outlier 抑制且满足角度门限）。"""
        if self._snr_drop_blocked() or self._last_angle is None:
            return

        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        tf = self._lookup_base_in(map_frame, base_frame)
        if tf is None or not self._true_local_aoa_ok(tf):
            return

        LL = float(self.get_parameter('trail_length').value)
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        base_yaw = self._yaw_from_quaternion(tf.transform.rotation)
        theta = base_yaw + self._last_angle

        p0 = Point(x=x, y=y, z=0.02)
        p1 = Point(x=x + LL * math.cos(theta), y=y + LL * math.sin(theta), z=0.02)

        # 容量控制（防爆）
        cap = int(self.get_parameter('trail_capacity').value)
        if cap <= 0:
            cap = 1
        if len(self._trails) >= cap:
            self._trails = self._trails[-(cap - 1):]  # 丢最旧

        self._trails.append((p0, p1, self._now_sec()))

    # ---------------- 按真实时间渐隐的轨迹构建 ----------------
    def _build_time_fading_trails(self) -> List[Marker]:
        """
        为每条历史线段基于 age 计算透明度：
            alpha(age) = trail_alpha_max * 0.5 ** ( age / half_life )
        超过 trail_max_age_sec 的线段会被删除，不再发布。
        """
        trail_width = float(self.get_parameter('trail_line_width').value)
        alpha_max = float(self.get_parameter('trail_alpha_max').value)
        half_life = max(1e-3, float(self.get_parameter('trail_fade_half_life_sec').value))
        max_age = max(half_life, float(self.get_parameter('trail_max_age_sec').value))

        now = self._now_sec()
        survivors: List[Tuple[Point, Point, float]] = []
        markers: List[Marker] = []

        for idx, (p0, p1, t_birth) in enumerate(self._trails):
            age = now - t_birth
            if age > max_age:
                continue  # 删除超龄
            # 指数衰减
            alpha = alpha_max * (0.5 ** (age / half_life))
            # 下限裁剪（避免过暗渲染开销），也可以让其直接删除
            if alpha < 0.02:
                continue

            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa_trails'
            m.id = 100 + idx  # 仅用于 RViz 缓存；索引变化时不影响显示
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.points = [p0, p1]
            m.scale.x = trail_width
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = float(alpha)
            m.lifetime = Duration(sec=0, nanosec=0)
            markers.append(m)
            survivors.append((p0, p1, t_birth))

        # 用剩余的覆盖（完成“删除超龄/超暗”）
        self._trails = survivors

        # （兼容处理：若数量下降，删除多余ID——通常无需，但保留以防 RViz 残影）
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
        """退出前清理所有 Marker。"""
        arr = MarkerArray()

        # 删除箭头
        m0 = Marker()
        m0.header.frame_id = self.get_parameter('map_frame').value
        m0.header.stamp = self.get_clock().now().to_msg()
        m0.ns = 'aoa'
        m0.id = 0
        m0.action = Marker.DELETE
        arr.markers.append(m0)

        # 删除 trails（按当前已知数量）
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
                # Topics
                'aoa_topic': '/aoa_angle',
                'snr_topic': '/snr_db',
                'marker_topic': '/aoa_markers',

                # TX position in the map frame
                'tx_x': 3.0,
                'tx_y': 5.0,

                # Thresholds
                'snr_threshold_db': 5.0,
                'aoa_abs_deg_limit': 30.0,

                # Visualization
                'arrow_length': 1.0,
                'trail_length': 10.0,
                'trail_period': 6.0,
                'trail_capacity': 20,
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



**Build**

```
cd ~/tb4_ws
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tb4_aoa_viz
source install/setup.bash

```

### Commends to bringup the visualization

when reboot the turtlebot, first need to delete a duplicate default gateway, after adding the required ip table

```
# update turtlebot ip route
sudo ip route del default via 10.33.16.1 dev wlan0
```

The ros2 use the fastDDS server, default at address 192.168.186.1, need to re-assign it, otherwise the fastDDS communication may fail. "ros2 topic list" cannot be seen.



#### solving the laptop - turtlebot communication problem on ROS2

**on laptop:** start a new fastDDS server 

one terminal: keep listening

```
fastdds discovery -i 0
# 默认监听 UDP 11811；窗口保持开启
```

another terminal

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811

source /opt/ros/jazzy/setup.bash
ros2 daemon stop
ros2 daemon cleanup
ros2 daemon start

```

**on turtlebot4**

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811

source /opt/ros/jazzy/setup.bash
ros2 daemon stop
ros2 daemon cleanup
ros2 daemon start
```

**backup**

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811   # laptop 的固定 IP
source /opt/ros/jazzy/setup.bash
```





#### solving the laptop - turtlebot time sync issue

**on laptop**:

```
sudo apt update
sudo apt install -y chrony

sudo nano /etc/chrony/chrony.conf
```

```

# comment all pool* then add:

# 允许该网段的客户端来对时
allow 192.168.185.0/24

# 无外网时，将本机作为低优先级时间源
local stratum 10

# 启动早期允许大步修正，避免开机后偏差过大
makestep 1.0 3
```

```
sudo systemctl restart chrony
sudo systemctl enable chrony
chronyc tracking
chronyc sources -v
```

**on turtlebot4**:

```
# turtlebot
sudo nano /etc/chrony/chrony.conf


# comment all pool*

server 192.168.185.2 iburst prefer
makestep 1.0 3
```

```
sudo systemctl restart chrony
sudo systemctl enable chrony
chronyc tracking
chronyc sources -v

```



### Commend for slam & Rviz

Terminal A1: SLAM

```
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch turtlebot4_navigation slam.launch.py
```

Terminal A2: RViz

```
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch turtlebot4_viz view_navigation.launch.py
```

Terminal A3: AOA visualization（tb4_aoa_viz）Draw the angle marker

```
source /opt/ros/jazzy/setup.bash
source ~/tb4_ws/install/setup.bash   # already installed
export ROS_DOMAIN_ID=0
ros2 launch tb4_aoa_viz aoa_marker.launch.py
```

Terminal A4: compute and publish AOA（TX→RX 的 LOS，publish to /aoa_angle）

```
source /opt/ros/jazzy/setup.bash
source ~/tb4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch tb4_aoa_viz aoa_tx2rx.launch.py
```

commend to set Tx location

```
ros2 param set /aoa_marker_node tx_x 3.27
ros2 param set /aoa_marker_node tx_y -1.45

```





### Commend for FPV camera stream

```
source /opt/ros/jazzy/setup.bash
rqt
```



mean 0, variance 3 degree every 100 ms new data.