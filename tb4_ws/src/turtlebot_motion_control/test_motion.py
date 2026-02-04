from map_motion_api import MapMotionAPI
import math
import time

api = MapMotionAPI(
        cmd_topic="/cmd_vel_unstamped",
        odom_topic="/odom",
        rate=10.0,
        max_linear=0.50,
        max_angular=0.25,
        source_frame="base_link",      # change to "base_footprint" if your TF uses that
        lin_accel_limit=0.05,
        ang_accel_limit=0.8,
    )

try:
    x, y, yaw = api.read_pos()
    # api.move(yaw=math.pi/2, distance=1.0)
    print(f"before x : {x} \n y : {y} \n yaw : {yaw}")
    
    time.sleep(2.0)
    
    
    api.move(yaw=0.0, distance=1.0)
    
    time.sleep(2.0)
    
    x, y, yaw = api.read_pos()
    print(f"middle x : {x} \n y : {y} \n yaw : {yaw}")
    
    
    
    api.move(yaw=math.pi, distance=1.0)
    
    time.sleep(2.0)
    
    x, y, yaw = api.read_pos()
    print(f"after x : {x} \n y : {y} \n yaw : {yaw}")
    
finally:
    api.shutdown()



# ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"