# TVMD Navigator

This project is built on ROS1 noetic on Ubuntu 20.04 arm64 (a Jetson Nano). 
The [ROS2 version](https://github.com/TVMD-UAV/TVMD-Navigator/tree/ros2) is successfully built, but the RealSense node failed to retrieve RGB camera frame. 


# Usage

## Prerequisites Installation
Target environment: 
- Ubuntu 20.04 on Jetson Nano with kernel version 4.9.253, which can be found [here](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image). Remember to resize the storage after flashing, which can be done using [Gparted](https://gparted.org/display-doc.php%3Fname%3Dmoving-space-between-partitions).
- ROS noetic
- Install [RealSense Backend](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
- Install [RealSense ROS1 wrapper](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file)
    ```bash
    sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
    ```

## Build
Clone the repo and install dependencies using [rosdep](https://wiki.ros.org/rosdep)
```bash
git clone https://github.com/TVMD-UAV/TVMD-Navigator.git
cd TVMD-Navigator/nav_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Run

Running the following command to launch imu listener and the camera node.
```bash
roslaunch cam_calibr cam_imu_start.launch
```
