# voxl_mpa_to_ros

ROSNode that takes in mpa data and published it to ROS

### Dependencies

* System Image 3.2 or later, [Instructions to Flash](https://docs.modalai.com/flash-system-image/)
* Camera, VIO and IMU servers, Installation: 
```
opkg update
opkg install voxl-camera-server
opkg install voxl-qvio-server
opkg install voxl-imu-server
```
  * [voxl-camera-server](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-camera-server)
  * [voxl-qvio-server](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-qvio-server)
  * [voxl-imu-server](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-imu-server)


### Installation
Install mpa-to-ros by installing the latest version of voxl-nodes
```
opkg install voxl-nodes
```

It is strongly recommended that you install voxl-mpa-tools while using mpa-to-ros as the library contains a plethora of useful tools to ensure that mpa is working properly.

```
opkg install voxl-mpa-tools
```


### Start Installed MPA ROS Node
```
bash
export ROS_IP=`hostname -i`
source /opt/ros/indigo/setup.bash
roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
```
##### Usage Instructions
In order for ros topics to actually appear, you must make sure that the relative
mpa server is started. You can use voxl-inspect-services from the mpa-tools library
to see a list of available services, and use opkg to install any that are not visible there.
You can start/stop these services at any point while mpa to ros is running and it will close
and open advertisements approprtiately. To start/stop these mpa services, you can use:
```
systemctl start voxl-camera-server
systemctl start voxl-imu-server
systemctl start voxl-qvio-server
```
or similarly replacing start with stop or camera server with another mpa server name. Additionally, 
you can run any of these servers in an ssh or adb window by typing their executable name i.e.
```
voxl-camera-server
```
Manually running these will require an open shell window, but will often have the ability to more 
easily see logged data from the servers.

##### Supported Interfaces
The current supported mpa->ros translations are:  

-Tracking and stereo cameras from voxl-camera-server  

-Imu0 and Imu1 from voxl-imu-server  

-VIO data from voxl-qvio-server (the data will appear under the qvio name, but it is normal vio data)  

### Expected Behavior
```
yocto:~# voxl-mpa-to-ros
... logging to /home/root/.ros/log/950b976a-839c-11eb-a1c1-ec5c68cd23bd/roslaunch-apq8096-22692.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.83:55583/

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
 * /mpa/voxl_mpa_to_ros_node/vio0_pipe: qvio
 * /mpa/voxl_mpa_to_ros_node/vio0_publish: True
 * /mpa/voxl_mpa_to_ros_node/vio1_pipe: 
 * /mpa/voxl_mpa_to_ros_node/vio1_publish: False
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /mpa/
    voxl_mpa_to_ros_node (voxl_mpa_to_ros/voxl_mpa_to_ros_node)

auto-starting new master
process[master]: started with pid [22711]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 950b976a-839c-11eb-a1c1-ec5c68cd23bd
process[rosout-1]: started with pid [22724]
started core service [/rosout]
process[mpa/voxl_mpa_to_ros_node-2]: started with pid [22741]
Param: "vio1_publish" set to false, not publishing associated interface


MPA to ROS app is now running

Starting Manager Thread with 5 interfaces

Found pipe for interface: tracking, now advertising
Did not find pipe for interface: stereo,
    interface will be idle until its pipe appears
Found pipe for interface: imu0, now advertising
Found pipe for interface: imu1, now advertising
Found pipe for interface: qvio, now advertising
```
