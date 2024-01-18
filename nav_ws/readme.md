# Odometry Publisher

## Installation
To build a certain package:
```bash
catkin_make_isolated --pkg=odometry_publisher
```

## Quick Start

```bash
sudo chmod 666 /dev/ttyTHS1
```

### AprilTag Based Pose Estimation
```bash
roslaunch odometry_publisher apriltag_start.launch
```

### VINS Mono Based Pose Estimation
```bash
roslaunch odometry_publisher odometry_start.launch
```