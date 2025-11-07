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
