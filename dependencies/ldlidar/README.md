# LD06 integration

The included launch file comes from [LDROBOT's ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_ros2) at commit `0f6101c93db0b4448cbe7a654826adcdee50622f`. The original robot configuration changes the serial port from `/dev/ttyUSB0` to `/dev/ttyAMA0`; the launch logic is otherwise unchanged. Both supplied local workspaces contain the same driver revision and adapted launch file.

Its SDK is [ldlidar_sdk](https://github.com/ldrobotSensorTeam/ldlidar_sdk), commit `6148cd3c33ae39adbd7e12faf82567fa8f8e7b0d`. Both are MIT licensed, copyright 2020 SHENZHEN LDROBOT CO., LTD.; their notices are retained as [LICENSE](LICENSE) and [SDK-LICENSE](SDK-LICENSE). These licenses apply to upstream driver material, not the custom RoverLink controller code.

The driver runs on the robot. This launch file selects LD06, 230400 baud, `/scan` (`sensor_msgs/LaserScan`), `/pointcloud2d` (`sensor_msgs/PointCloud`), and frame `base_laser`. It also publishes a static `base_link` → `base_laser` transform with z = 0.18 m. Verify that sensor mounting and serial device match the actual robot.

See [robot setup](../../robot/README.md#ld06-driver) for the pinned build and launch commands. The full upstream source and its Git history are not vendored here.
