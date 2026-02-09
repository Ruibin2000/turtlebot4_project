### Turtlebot motion Control api

**Always first initialize the position and start the slam**

```
source /opt/ros/jazzy/setup.bash
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"
ros2 launch turtlebot4_navigation slam.launch.py
```



In the folder src/turtlebot_motion_control

```
turtlebot_motion_control	(All required)
	-- map_motion_api.py	# the api 
	-- motion_simple.py		# the ros2 move implementation, called by map_motion_api.py
	-- test_motion.py		# demo how to use api to control movement
```

How to use api:

```
# first initialize the api object
from map_motion_api import MapMotionAPI
import math
import time

api = MapMotionAPI(
        cmd_topic="/cmd_vel_unstamped",
        odom_topic="/odom",
        rate=10.0,
        max_linear=0.50,
        max_angular=0.25,
        source_frame="base_link",
        lin_accel_limit=0.05,
        ang_accel_limit=0.8,
    )


"""
map_motion_api.py

A clean, importable API for TurtleBot motion using TF (map->base) + cmd_vel.

Public API:
  - read_pos() -> (x, y, yaw) in map frame
  - move(yaw, distance) : rotate to absolute yaw in map, then forward distance (m)
  - compute_yaw_distance_to_target ([cur_x,cur_y], [tgt_x, tgt_y]): calculate the yaw and distance moving to the target from current

Requires:
  - motion_simple.py providing MotionSimple and normalize_angle
"""
```

