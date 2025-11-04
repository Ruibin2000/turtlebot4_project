#!/usr/bin/env python3
import time
import math
import random
import argparse

from tb4_aoa_viz.aoa_bridge import get_publish_aoa_fn
from tb4_aoa_viz.snr_bridge import get_publish_snr_fn


def compute_aoa() -> float:
    """Simulate AOA in radians, uniformly drawn from [-pi, +pi]."""
    return random.uniform(-math.pi, math.pi)


def read_snr_db() -> float:
    """Simulate SNR in dB, uniformly drawn from [5, 35] dB."""
    return random.uniform(5.0, 35.0)


def main():
    parser = argparse.ArgumentParser(description="Publish AOA (rad) and SNR (dB) at a fixed rate.")
    parser.add_argument("--aoa_topic", default="/aoa_angle", help="AOA topic (Float32, radians)")
    parser.add_argument("--snr_topic", default="/snr_db", help="SNR topic (Float32, dB)")
    parser.add_argument("--rate_hz", type=float, default=10.0, help="Publish rate in Hz")
    parser.add_argument("--no_print", action="store_true", help="Disable console prints")
    args = parser.parse_args()

    # Initialize bridges (each sets up its own ROS2 node/executor internally)
    publish_aoa = get_publish_aoa_fn(args.aoa_topic)
    publish_snr = get_publish_snr_fn(args.snr_topic)

    period = 1.0 / max(1e-6, args.rate_hz)
    next_time = time.time()

    if not args.no_print:
        print(f"Publishing AOA (rad) → {args.aoa_topic} "
              f"and SNR (dB) → {args.snr_topic} at {args.rate_hz:.2f} Hz ...")

    try:
        while True:
            aoa = compute_aoa()
            snr = read_snr_db()

            # one compute/read → one publish (each loop)
            publish_aoa(aoa)
            publish_snr(snr)

            if not args.no_print:
                print(f"AOA = {aoa:+.3f} rad ({math.degrees(aoa):6.1f}°),  SNR = {snr:5.2f} dB")

            # maintain loop rate
            next_time += period
            sleep_dt = next_time - time.time()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                # if we overrun, reset the schedule to avoid drift
                next_time = time.time()
    except KeyboardInterrupt:
        if not args.no_print:
            print("\nStopped publishing.")


if __name__ == "__main__":
    main()
