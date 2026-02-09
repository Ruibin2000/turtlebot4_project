# Turtlebot4_project

This is the turtlebot TX localization project for Brooklyn 6G summit. Following demo shows how the turtlebot using triangle localization while running slam to construct the map.

[![Turtlebot Demo](https://img.youtube.com/vi/ZR88qGvGCt0/hqdefault.jpg)](https://youtu.be/ZR88qGvGCt0)



This is the set up file backup repo for turtlebot4 project, including two parts

1. Configure the support ros2 publisher to draw the slam marker & aoa publisher
2. Commends to bringup the visualization



**Create workspace**

First create the workspace or copy the tb4_ws folder to the local

**Build the workspace**

```
# path to the tb4_ws, costomized workspace
cd ~/Desktop/turtlebot4_github/turtlebot4_project/tb4_ws
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

sudo ip route add default via 10.33.16.1 dev wlan0
```

The ros2 use the fastDDS server, default at address 192.168.186.1, need to re-assign it, otherwise the fastDDS communication may fail. "ros2 topic list" cannot be seen.



#### solving the laptop - turtlebot communication problem on ROS2

**Using the cycloneDDS**: unicast, tcp

on laptop:

```
fastdds discovery -i 0 -p 11811
# default fast dds server port 11811
```

another terminal



```
source /opt/ros/jazzy/setup.bash
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
unset ROS_DISCOVERY_SERVER
```



```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
#unset ROS_LOCALHOST_ONLY
#unset CYCLONEDDS_URI
export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811
ros2 daemon stop
ros2 daemon start

```

on turtlebot4:

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
#export FASTDDS_DISCOVERY_SERVER=192.168.0.224:11811

```




**both restart domain:**

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds/cyclonedds.xml
unset FASTDDS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY

ros2 daemon stop
ros2 daemon start

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

# allow client check
allow 192.168.185.0/24

local stratum 10

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

turtlebot4 bringup:

```
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_bringup robot.launch.py publish_tf:=true
```



```
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros tf2_echo odom base_link
```



Terminal A1: SLAM

```
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch turtlebot4_navigation slam.launch.py


source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_navigation slam.launch.py \
	use_sim_time:=false \
	autostart:=true
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
source ~/Desktop/turtlebot4_github/turtlebot4_project/tb4_ws/install/setup.bash   # already installed
export ROS_DOMAIN_ID=0
ros2 launch tb4_aoa_viz aoa_marker.launch.py
```

Terminal A4: compute and publish AOA（TX→RX 的 LOS，publish to /aoa_angle）

```
source /opt/ros/jazzy/setup.bash
source ~/Desktop/turtlebot4_github/turtlebot4_project/tb4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch tb4_aoa_viz aoa_tx2rx.launch.py
```

commend to set Tx location

```
ros2 param set /aoa_marker_node tx_x 3.27
ros2 param set /aoa_marker_node tx_y -1.45

```

### Commend for Reset Odom location

```
source /opt/ros/jazzy/setup.bash
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"
ros2 launch turtlebot4_navigation slam.launch.py


# check position
ros2 run tf2_ros tf2_echo map base_link
```





### Commend to control bot moving

```
# first run the slam and initialize the odom location

source /opt/ros/jazzy/setup.bash
python3 test_motion.py 

```

following are the demo code:



```
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
```


### Commend for PTU rotation


```
python3 test_ptu.py

```




### Commend for FPV camera stream

```
source /opt/ros/jazzy/setup.bash
rqt
```
