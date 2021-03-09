# voxl_mpa_to_ros

ROSNode that takes in mpa data and published it to ROS

#### Start Installed TOF ROS Node
```
bash
export ROS_IP=`hostname -i`
export TOF_CAM_ID=-1
source /opt/ros/indigo/setup.bash
roslaunch /opt/ros/indigo/share/voxl_mpa_to_ros/launch/voxl_mpa_to_ros.launch
```

- When running a custom build of the package use (assuming it's in `/home/root/git/voxl-hal3-tof-cam-ros`):
#### Start Locally Built TOF ROS Node
```
# this will re-build the code and launch the app
bash
export ROS_IP=`hostname -i`
export TOF_CAM_ID=-1
cd /home/root/git/voxl_mpa_to_ros
./clean.sh
./build.sh
source ./devel/setup.bash
roslaunch ./source/launch/mpa.launch
```

### Expected Behavior
```
yocto:/# roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
... logging to /home/root/.ros/log/5924d82e-7d43-11eb-b88d-ec5c68cd23bd/roslaunch-apq8096-3551.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.83:55783/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /mpa/
    voxl_mpa_to_ros_node (voxl_mpa_to_ros/voxl_mpa_to_ros_node)

auto-starting new master
process[master]: started with pid [3570]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 5924d82e-7d43-11eb-b88d-ec5c68cd23bd
process[rosout-1]: started with pid [3583]
started core service [/rosout]
process[mpa/voxl_mpa_to_ros_node-2]: started with pid [3592]


MPA to ROS app is now running

Starting Manager Thread with 2 interfaces
Interface 0 now advertising
Interface 1 now advertising
Interface 1 now publishing

```