# voxl_mpa_to_ros

ROSNode that takes in mpa data and published it to ROS

## Build and install the project using `voxl-emulator`:

- Clone project:

```bash
git clone https://gitlab.com/voxl-public/ros/voxl_mpa_to_ros
cd voxl_mpa_to_ros
```

- Run voxl-emulator docker
  - Note: information regarding the voxl-emulator can be found [here](https://gitlab.com/voxl-public/voxl-docker)

```bash
voxl-docker -i voxl-emulator
```

- Build project binary:

```bash
./install_build_deps.sh
source /opt/ros/indigo/setup.bash
./build.sh source/
```

This will generate a binary in "devel/lib/voxl_hal3_tof_cam_ros/voxl_hal3_tof_cam_ros_node"

- Build the IPK

```bash
./make_package.sh
```

- If VOXL is connected via adb, you can install on  target;

```bash
./install_on_voxl.sh
```

## Build and install the project directly on `VOXL`:
```
bash
mkdir -p /home/root/git
cd /home/root/git
git clone https://gitlab.com/voxl-public/ros/voxl_mpa_to_ros
cd voxl_mpa_to_ros
./install_build_deps.sh
./build.sh
./make_package.sh
opkg install ./voxl-hal3-tof-cam-ros_0.0.3.ipk
```

## Run TOF Application on VOXL

### Determine VOXL IP Address
- For WiFi setup, instructions found [here](https://docs.modalai.com/wifi-setup/)
- Run `ifconfig` in a terminal
- Check `inet addr` value for `wlan0` (if using wifi), this will give you the IP address of VOXL
- `hostname -i` also works

### Start TOF ROS Node
- If needed, open a new terminal on VOXL (use adb or ssh)
- Make sure your terminal is using bash (when in doubt just run `bash` after opening the terminal)
- Identify the correct camera ID to use for TOF sensor
   - For camera id please check [here](https://docs.modalai.com/camera-connections/#configurations)
- Export TOF_CAM_ID in your environment before running the TOF application, for example
   - `export TOF_CAM_ID=1`
      - starting with release 0.0.3, you can choose to autodetect TOF camera id
      - `export TOF_CAM_ID=-1` will tell the application to attempt to autodetect TOF camera id
- By default all outputs are enabled i.e. IR-Image, Depth-Image, Point-Cloud (see `tof.launch`)

- When running package installed to `/opt/ros/indigo` use:

#### Start Installed TOF ROS Node
```
bash
export ROS_IP=`hostname -i`
export TOF_CAM_ID=-1
source /opt/ros/indigo/setup.bash
roslaunch /opt/ros/indigo/share/voxl_mpa_to_ros/launch/mpa.launch
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
- the very first time the sensor is used, it will be initialized (about 30 seconds)
- lens parameters will be downloaded from sensor if needed to `/data/misc/camera/irs10x0c_lens.cal`
```
yocto:/# roslaunch /opt/ros/indigo/share/voxl_mpa_to_ros/launch/mpa.launch
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