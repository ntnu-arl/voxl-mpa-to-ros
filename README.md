# voxl_mpa_to_ros

ROSNode that takes in mpa data and published it to ROS

#### Start Installed MPA ROS Node
```
bash
export ROS_IP=`hostname -i`
source /opt/ros/indigo/setup.bash
roslaunch /opt/ros/indigo/share/voxl_mpa_to_ros/launch/voxl_mpa_to_ros.launch
```

### Expected Behavior
```
... logging to /home/root/.ros/log/8509e9ee-81ca-11eb-b46d-ec5c68cd23bd/roslaunch-apq8096-3794.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.83:48070/

SUMMARY
========

PARAMETERS
 * /mpa/voxl_mpa_to_ros_node/imu0_pipe: imu0
 * /mpa/voxl_mpa_to_ros_node/imu0_publish: True
 * /mpa/voxl_mpa_to_ros_node/imu1_pipe: imu1
 * /mpa/voxl_mpa_to_ros_node/imu1_publish: True
 * /mpa/voxl_mpa_to_ros_node/stereo_pipe: stereo
 * /mpa/voxl_mpa_to_ros_node/stereo_publish: True
 * /mpa/voxl_mpa_to_ros_node/tracking_pipe: tracking
 * /mpa/voxl_mpa_to_ros_node/tracking_publish: True
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /mpa/
    voxl_mpa_to_ros_node (voxl_mpa_to_ros/voxl_mpa_to_ros_node)

auto-starting new master
process[master]: started with pid [3813]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 8509e9ee-81ca-11eb-b46d-ec5c68cd23bd
process[rosout-1]: started with pid [3826]
started core service [/rosout]
process[mpa/voxl_mpa_to_ros_node-2]: started with pid [3837]


MPA to ROS app is now running

Starting Manager Thread with 4 interfaces
Interface tracking now advertising
Interface stereo now advertising
Interface imu0 now advertising
Interface imu1 now advertising
```
