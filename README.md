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
import math
from collections import deque
from typing import Optional, Deque, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration


class AOAMarkerNode(Node):
    def __init__(self):
        super().__init__('aoa_marker_node')

        # ---- Parameters ----
        self.declare_parameter('aoa_topic', '/aoa_angle')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('arrow_length', 1.0)
        self.declare_parameter('shaft_diameter', 0.03)
        self.declare_parameter('head_diameter', 0.08)
        self.declare_parameter('head_length', 0.15)
        self.declare_parameter('angle_in_degrees', False)
        self.declare_parameter('marker_topic', '/aoa_markers')

        # Long-ray (trail) params
        self.declare_parameter('trail_topic', '/aoa_trails')
        self.declare_parameter('trail_length', 10.0)          # 长线长度
        self.declare_parameter('trail_line_width', 0.02)       # 线宽(scale.x)
        self.declare_parameter('trail_period', 3.0)            # 每隔多少秒采样一次
        self.declare_parameter('trail_capacity', 20)           # 历史线总数
        self.declare_parameter('trail_recent_fade_count', 10)  # 最新N条渐隐
        self.declare_parameter('trail_alpha_min', 0.08)        # 很淡
        self.declare_parameter('trail_alpha_max', 0.90)        # 最亮（最新）

        aoa_topic = self.get_parameter('aoa_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self._trail_topic = self.get_parameter('trail_topic').value

        # ---- ROS I/O ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._aoa_sub = self.create_subscription(Float32, aoa_topic, self._aoa_cb, 10)
        # 统一用 MarkerArray 发布（RViz 对 Marker 和 MarkerArray 都支持）
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        # 历史长线也走同一个 topic（一个 MarkerArray 里可同时带箭头和线）
        # 也可以分 topic；为简单起见同一发布器即可
        # self._trail_pub = self.create_publisher(MarkerArray, self._trail_topic, 10)

        self._last_angle: Optional[float] = None
        self._last_tf_ok = False

        # 历史线：保存为 (p0, p1) 的 deque
        self._trails: Deque[Tuple[Point, Point]] = deque(maxlen=self.get_parameter('trail_capacity').value)

        # 定时器：箭头刷新 + 采样长线
        self._timer = self.create_timer(0.1, self._tick_arrow)
        self._trail_timer = self.create_timer(self.get_parameter('trail_period').value, self._sample_trail)

        self.get_logger().info(f'AOA marker listening on {aoa_topic}, publishing {marker_topic}')

    # ---------- Callbacks ----------
    def _aoa_cb(self, msg: Float32):
        angle = math.radians(msg.data) if self.get_parameter('angle_in_degrees').value else msg.data
        self._last_angle = angle

    # ---------- Helpers ----------
    def _yaw_from_quaternion(self, q):
        # ZYX yaw from quaternion
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        return math.atan2(t3, t4)

    def _lookup_base_in(self, map_frame: str, base_frame: str):
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
            self._last_tf_ok = True
            return tf
        except Exception as e:
            if self._last_tf_ok:
                self.get_logger().warn(f'lookup_transform({map_frame}->{base_frame}) failed once: {e}')
            self._last_tf_ok = False
            return None

    # ---------- Publishing ----------
    def _tick_arrow(self):
        """实时箭头（ARROW）"""
        if self._last_angle is None:
            return

        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        L = self.get_parameter('arrow_length').value
        shaft_d = self.get_parameter('shaft_diameter').value
        head_d = self.get_parameter('head_diameter').value
        head_L = self.get_parameter('head_length').value

        tf = self._lookup_base_in(map_frame, base_frame)
        if tf is None:
            return

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
        arrow.id = 0  # 固定 ID（覆盖更新）
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
        arrow.lifetime = Duration(sec=0, nanosec=0)  # 永久

        arr = MarkerArray()
        arr.markers.append(arrow)

        # 也顺带把历史线刷新（颜色渐隐需要持续更新）
        arr.markers.extend(self._build_trail_markers(map_frame))

        self._marker_pub.publish(arr)

    def _sample_trail(self):
        """每 trail_period 采样一次，生成一条长线并加入历史"""
        if self._last_angle is None:
            return

        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        LL = self.get_parameter('trail_length').value

        tf = self._lookup_base_in(map_frame, base_frame)
        if tf is None:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        base_yaw = self._yaw_from_quaternion(tf.transform.rotation)
        theta = base_yaw + self._last_angle

        p0 = Point(x=x, y=y, z=0.02)  # 线稍微低一点避免盖住箭头
        p1 = Point(x=x + LL * math.cos(theta), y=y + LL * math.sin(theta), z=0.02)

        self._trails.append((p0, p1))

        # 立即发一次（可选，或等下一次箭头刷新统一发）
        arr = MarkerArray()
        arr.markers.extend(self._build_trail_markers(map_frame))
        self._marker_pub.publish(arr)

    def _build_trail_markers(self, frame_id: str):
        """根据 _trails 生成 20 条 LINE_LIST，并设置渐隐颜色"""
        trail_width = self.get_parameter('trail_line_width').value
        cap = self.get_parameter('trail_capacity').value
        recent_n = min(self.get_parameter('trail_recent_fade_count').value, cap)
        alpha_min = self.get_parameter('trail_alpha_min').value
        alpha_max = self.get_parameter('trail_alpha_max').value

        markers = []

        n = len(self._trails)
        # 为了稳定 ID 映射：最老的放小 ID，最新的放大 ID
        # 我们使用 id = 100 + idx，避免与箭头 id=0 冲突
        for idx, (p0, p1) in enumerate(self._trails):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa_trails'
            m.id = 100 + idx
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.points = [p0, p1]
            m.scale.x = trail_width  # LINE_LIST 只用 scale.x 作为线宽

            # 颜色：后半段（最老的那部分）固定很淡；最新的 recent_n 条做线性渐隐
            # deque: 0..n-1 = old..new
            # old_count = max(0, n - recent_n)
            old_count = max(0, n - recent_n)
            if idx < old_count:
                alpha = alpha_min  # 老的：恒定很淡
            else:
                # 在最近 recent_n 内做渐隐：
                # 令 rank 从 0..recent_n-1（越新 rank 越大），alpha 越高
                rank = idx - old_count
                if recent_n > 1:
                    t = (rank) / (recent_n - 1)
                else:
                    t = 1.0
                alpha = alpha_min + (alpha_max - alpha_min) * t  # 线性插值

            # 颜色你可自定义，这里用橙色系
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = float(alpha)

            m.lifetime = Duration(sec=0, nanosec=0)  # 永久存在，靠重复发布覆盖颜色
            markers.append(m)

        # 如果曾经发布过更多条（比如从 20 缩到 15），需要删除多余的 ID
        # 用 DELETE 动作把多余 ID 清理掉
        if n < self._last_published_trail_count:
            for idx in range(n, self._last_published_trail_count):
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'aoa_trails'
                m.id = 100 + idx
                m.action = Marker.DELETE
                markers.append(m)

        self._last_published_trail_count = n
        return markers

    # ---------- ROS2 lifecycle ----------
    def on_shutdown(self):
        # 清掉所有 trail
        arr = MarkerArray()
        # DELETEALL 不是所有版本都支持，稳妥起见逐个 DELETE
        for idx in range(self._last_published_trail_count):
            m = Marker()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aoa_trails'
            m.id = 100 + idx
            m.action = Marker.DELETE
            arr.markers.append(m)
        # 也删掉箭头
        m = Marker()
        m.header.frame_id = self.get_parameter('map_frame').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aoa'
        m.id = 0
        m.action = Marker.DELETE
        arr.markers.append(m)

        self._marker_pub.publish(arr)

    # 初始化之后补一个字段
    _last_published_trail_count: int = 0


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
            parameters=[
                {'angle_in_degrees': False},
                {'aoa_topic': '/aoa_angle'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'arrow_length': 1.0},
                {'shaft_diameter': 0.03},
                {'head_diameter': 0.08},
                {'head_length': 0.15},
                {'marker_topic': '/aoa_markers'},
            ]
        )
    ])

```

**Next, create the aoa publisher**, replace it with the antenna aoa receiver program

provide the true aoa publisher interface

```
nano ~/tb4_ws/src/tb4_aoa_viz/tb4_aoa_viz/aoa_bridge.py

```

```
import threading
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32


class _AOANode(Node):
    """Internal ROS2 node that owns the /aoa_angle publisher."""
    def __init__(self, topic: str = "/aoa_angle", queue_size: int = 10):
        super().__init__("aoa_bridge_node")
        self._pub = self.create_publisher(Float32, topic, queue_size)

    def publish_angle(self, angle_rad: float) -> None:
        msg = Float32()
        msg.data = float(angle_rad)
        self._pub.publish(msg)
        # self.get_logger().debug(f"Published AOA: {msg.data:.6f} rad")


class AOAPublisher:
    """Public interface (callee): expose publish(angle_rad)."""
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
class _PublishFnHolder:
    _lock = threading.Lock()
    _publisher: Optional[AOAPublisher] = None

    @classmethod
    def get_publish_fn(cls, topic: str = "/aoa_angle") -> Callable[[float], None]:
        with cls._lock:
            if cls._publisher is None:
                cls._publisher = AOAPublisher(topic=topic)
                cls._publisher.start()

        def _publish(angle_rad: float) -> None:
            assert cls._publisher is not None
            cls._publisher.publish(angle_rad)

        return _publish


def get_publish_fn(topic: str = "/aoa_angle") -> Callable[[float], None]:
    return _PublishFnHolder.get_publish_fn(topic=topic)

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
ros2 daemon stop; ros2 daemon start

```

**on turtlebot4**

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811

source /opt/ros/jazzy/setup.bash
ros2 daemon stop; ros2 daemon start
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







mean 0, variance 3 degree every 100 ms new data.