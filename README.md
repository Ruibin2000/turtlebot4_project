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
nano ~/.cyclonedds/cyclonedds.xml
```

```
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>

    <Discovery>
      <Peers>
        <Peer address="192.168.185.3"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>

```

on turtlebot4:

```
nano ~/.cyclonedds/cyclonedds.xml
```

```
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <!-- 先允许 multicast，保证本机不会被锁死 -->
      <AllowMulticast>true</AllowMulticast>
    </General>

    <Discovery>
      <Peers>
        <!-- 注意：不要 udp:// 前缀 -->
        <Peer address="192.168.0.224"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>

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
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"
ros2 launch turtlebot4_navigation slam.launch.py


# check position
ros2 run tf2_ros tf2_echo map base_link
```



### Commend to control bot moving

```
# first run the slam


source /opt/ros/jazzy/setup.bash
python3 test_motion.py 

```









### Commend for FPV camera stream

```
source /opt/ros/jazzy/setup.bash
rqt
```
