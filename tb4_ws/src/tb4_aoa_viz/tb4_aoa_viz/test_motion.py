#!/usr/bin/env python3
# test_motion.py
#
# 在 map 坐标系下控制 TurtleBot：
# 1. 旋转到指定绝对 yaw
# 2. 沿当前朝向行走指定距离

import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from motion_simple import MotionSimple, normalize_angle

from tf2_ros import Buffer, TransformListener, TransformException


# ================== TF 工具函数 ================== #

def wait_for_tf(tf_buffer: Buffer,
                executor,
                target_frame="map",
                source_frame="base_link",
                timeout=10.0):

    print(f"Waiting for TF {target_frame} -> {source_frame} ...")
    t0 = time.time()

    while time.time() - t0 < timeout:
        executor.spin_once(timeout_sec=0.1)
        try:
            tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            print("TF available.")
            return True
        except TransformException:
            pass

    print("TF timeout.")
    return False


def get_xy_yaw_from_tf(tf_buffer: Buffer,
                       target="map",
                       source="base_link"):

    try:
        trans = tf_buffer.lookup_transform(
            target,
            source,
            rclpy.time.Time()
        )
    except TransformException:
        return None

    t = trans.transform.translation
    q = trans.transform.rotation

    yaw = math.atan2(
        2*(q.w*q.z + q.x*q.y),
        1 - 2*(q.y*q.y + q.z*q.z)
    )

    return t.x, t.y, yaw


def get_stable_pose(tf_buffer: Buffer,
                    duration=1.0,
                    rate=30.0):

    poses = []
    t0 = time.time()
    dt = 1.0 / rate

    while time.time() - t0 < duration:
        pose = get_xy_yaw_from_tf(tf_buffer)
        if pose:
            poses.append(pose)
        time.sleep(dt)

    if not poses:
        return None

    xs, ys, yaws = zip(*poses)
    return sum(xs)/len(xs), sum(ys)/len(ys), sum(yaws)/len(yaws)


# ================== 在 map 下旋转 ================== #

def rotate_to_yaw_absolute_map(motion: MotionSimple,
                               executor,
                               tf_buffer: Buffer,
                               target_yaw: float,
                               base_angular_speed=0.3,
                               safety_timeout=15.0):

    Kp = 2.0
    min_w = 0.05
    max_w = min(base_angular_speed, motion.max_angular)
    stop_tol = math.radians(1.0)

    print(f"[map-rot] Rotating to yaw={target_yaw:.3f}")

    t0 = time.time()
    while True:
        executor.spin_once(timeout_sec=0.01)

        pose = get_xy_yaw_from_tf(tf_buffer)
        if pose is None:
            continue

        x, y, yaw = pose

        err = normalize_angle(target_yaw - yaw)
        print(f"\r[map-rot] yaw={yaw:.3f}, err={err:.3f}", end="")

        if abs(err) < stop_tol:
            print("\n[map-rot] Rotation complete.")
            break

        w = Kp * err
        w = max(min(w, max_w), -max_w)

        if abs(w) < min_w:
            w = min_w if w > 0 else -min_w

        motion.set_cmd(0.0, w)

        if time.time() - t0 > safety_timeout:
            print("\n[map-rot] Safety timeout.")
            break

    motion.stop()
    time.sleep(0.5)


# ================== 在 map 下前进 ================== #

def move_forward_distance_map(motion: MotionSimple,
                              executor,
                              tf_buffer: Buffer,
                              distance: float,
                              max_speed=0.10,
                              safety_timeout=40.0):

    pose0 = get_xy_yaw_from_tf(tf_buffer)
    if pose0 is None:
        print("Cannot read TF, cannot move.")
        return

    x0, y0, yaw0 = pose0

    print(f"[map] Start forward: x={x0:.3f}, y={y0:.3f}, yaw={yaw0:.3f}")

    Kv = 0.2
    Kw_yaw = 2.0
    Kw_cross = 0.5

    min_speed = 0.03
    max_speed = min(max_speed, motion.max_linear)
    max_w = motion.max_angular

    stop_tol = 0.001

    t0 = time.time()

    while True:
        executor.spin_once(timeout_sec=0.01)

        pose = get_xy_yaw_from_tf(tf_buffer)
        if pose is None:
            continue

        x, y, yaw = pose

        dx = x - x0
        dy = y - y0

        forward = dx*math.cos(yaw0) + dy*math.sin(yaw0)
        cross   = -dx*math.sin(yaw0) + dy*math.cos(yaw0)

        remaining = distance - forward

        print(f"\r[map] fwd={forward:.3f}/{distance:.3f}, cross={cross:.3f}", end="")

        if remaining <= stop_tol:
            print("\n[map] Forward reached target.")
            break

        # 线速度
        v = Kv * remaining
        v = max(min(v, max_speed), min_speed)

        # 角速度：保持朝向 yaw0
        yaw_err = normalize_angle(yaw0 - yaw)
        w = Kw_yaw*yaw_err + Kw_cross*cross
        w = max(min(w, max_w), -max_w)

        motion.set_cmd(v, w)

        if time.time() - t0 > safety_timeout:
            print("\n[map] Forward safety timeout.")
            break

    motion.stop()

    time.sleep(2.0)

    pose_end = get_stable_pose(tf_buffer, duration=1.0, rate=30.0)
    if pose_end:
        x1, y1, yaw1 = pose_end
        print(f"[map] End pose: x={x1:.3f}, y={y1:.3f}, yaw={yaw1:.3f}")

        straight = math.hypot(x1 - x0, y1 - y0)
        print(f"[map] Straight-line distance ≈ {straight:.3f} m")

    time.sleep(1.0)


# ================== 高层动作序列 ================== #
def run_sequence(first_turn_rad: float,
                 forward_distance_m: float):

    rclpy.init()

    motion = MotionSimple(
        max_linear=0.10,
        max_angular=0.25,
        cmd_topic="/cmd_vel_unstamped",
        odom_topic="/odom",
        rate=10.0,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(motion)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, motion)

    try:
        # 等 TF map->base_link
        if not wait_for_tf(tf_buffer, executor,
                           target_frame="map",
                           source_frame="base_link",
                           timeout=20.0):
            return

        # ==== 1) 根据“当前 yaw 和目标 yaw 的差”来决定要不要转 ====
        pose = get_xy_yaw_from_tf(tf_buffer, "map", "base_link")
        if pose is None:
            print("[seq] Cannot get current yaw from TF.")
        else:
            _, _, yaw_now = pose
            yaw_err = normalize_angle(first_turn_rad - yaw_now)
            print(f"[seq] current yaw={yaw_now:.3f}, "
                  f"target={first_turn_rad:.3f}, err={yaw_err:.3f}")

            # 只有“误差”足够大才去旋转，比如大于 1°
            if abs(yaw_err) > math.radians(1.0):
                rotate_to_yaw_absolute_map(
                    motion,
                    executor,
                    tf_buffer,
                    target_yaw=first_turn_rad,
                    base_angular_speed=0.3,
                    safety_timeout=20.0
                )
            else:
                print("[seq] Already near target yaw, skip rotation.")

        # ==== 2) 在 map 下前进 ====
        if forward_distance_m > 1e-4:
            move_forward_distance_map(
                motion,
                executor,
                tf_buffer,
                distance=forward_distance_m,
                max_speed=0.1,
                safety_timeout=60.0
            )

        print("\n=== Sequence complete ===")

    finally:
        executor.shutdown()
        motion.destroy_node()
        rclpy.shutdown()



# ================== main ================== #

def main(args=None):
    run_sequence(
        first_turn_rad=math.pi*0.000,     # 在 map 下的绝对 yaw
        forward_distance_m=0.2  # 在 map 下走 1m
    )


if __name__ == "__main__":
    main()
