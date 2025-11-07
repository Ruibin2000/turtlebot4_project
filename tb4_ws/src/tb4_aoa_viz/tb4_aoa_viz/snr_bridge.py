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
