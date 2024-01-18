# Camera-IMU Launcher

There are two different source for IMU:
1. I2C readings from the ESP32 (the [avionic board](https://github.com/TVMD-UAV/TVMD-Agent-Avionics), which acts as the navigator server.)
2. The IMU from Pixhawk through MAVLINK on `/mavlink/imu/data` or `/mavlink/imu/data_raw` (upto 160 Hz). (The PX4 needs to configure the external vision mode through MAVLINK to reach high frame rate)

Remember to change permission of `/dev/ttyTHS1` before launch mavros (I don't know whether it has a better solution.)
```bash
sudo chmod 666 /dev/ttyTHS1
```

Launch the nodes:
```bash
# Launch IMU listener from ESP32
roslaunch cam_calibr imu_start.launch

# Launch IMU listener from Pixhawk
roslaunch cam_calibr px4imu_start.launch

# Launch RealSense node + IMU listener from ESP32
roslaunch cam_calibr cam_imu_start.launch

# Launch RealSense node + IMU listener from Pixhawk
roslaunch cam_calibr cam_px4imu_start.launch
```

# Trouble Shooting

## Serial Port Permission Deny
If you have output like
```
process[mavros-2]: started with pid [33841]
[ INFO] [1704953439.207649885]: FCU URL: /dev/ttyTHS1:57600
[ INFO] [1704953439.216343247]: serial0: device: /dev/ttyTHS1 @ 57600 bps
[FATAL] [1704953439.217075398]: FCU: DeviceError:serial:open: Permission denied
================================================================================REQUIRED process [mavros-2] has died!
process has finished cleanly
log file: /home/jetson/.ros/log/235a7826-b048-11ee-abf2-19579c9dbe03/mavros-2*.log
Initiating shutdown!
================================================================================
```

According to [this](https://answers.ros.org/question/220319/ros-run-mavros-with-permission-denied/), to allow current user to have the read/write permission on `/dev/ttyTHS1`, add current user to `dialout` group. 
```bash
sudo gpasswd -a $USER dialout
```
Reboot the system and check user gourp by `$ groups`

Another solution:
```bash
sudo chmod 666 /dev/ttyTHS1
```