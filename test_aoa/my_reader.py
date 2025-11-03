import time
import math
import random
from tb4_aoa_viz.aoa_bridge import get_publish_fn

def computeAOA():
    """
    Simulate reading an AOA angle from antenna.
    Here we just return a random angle between -pi and +pi.
    """
    return random.uniform(-math.pi, math.pi)

def main():
    # Get publisher function from bridge (this initializes ROS2 node internally)
    publish_aoa = get_publish_fn("/aoa_angle")

    # 10 Hz loop
    rate_hz = 10.0
    period = 1.0 / rate_hz
    next_time = time.time()

    print("Publishing random AOA values to /aoa_angle at 10 Hz ...")
    try:
        while True:
            angle = computeAOA()
            publish_aoa(angle)  # one compute → one publish
            print(f"Published AOA: {math.degrees(angle):.2f}°  ({angle:.3f} rad)")
            # keep 10 Hz timing
            next_time += period
            sleep_dt = next_time - time.time()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.time()
    except KeyboardInterrupt:
        print("\nStopped publishing.")

if __name__ == "__main__":
    main()
