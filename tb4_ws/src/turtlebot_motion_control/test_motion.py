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
    cur_x, cur_y, yaw = api.read_pos()
    # api.move(yaw=math.pi/2, distance=1.0)
    print(f"before x : {cur_x} \n y : {cur_y} \n yaw : {yaw}")
    
    time.sleep(2.0)
    
    
    
    tgt_pos = [0.0,0.0]
    
    mv_yaw, mv_dis = api.compute_yaw_distance_to_target([cur_x,cur_y], tgt_pos)
    api.move(yaw=mv_yaw, distance=mv_dis)
    api.move(yaw=0.0, distance=0.0)
    
    time.sleep(2.0)
    
    x, y, yaw = api.read_pos()
    print(f"middle x : {x} \n y : {y} \n yaw : {yaw}")

finally:
    api.shutdown()



# ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"