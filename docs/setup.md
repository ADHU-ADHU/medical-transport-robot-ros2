# Setup and operation

## Environment

The original platform uses ROS 2 Humble on a PC and Raspberry Pi 4, with MD25/EMG30 drive, MPU6050 and an LD LiDAR configured through the LD06 launch profile. Keep both machines on the same ROS distribution and network for reproduction. A local Jazzy build check is recorded in [validation](validation.md); it is not a robot test.

Clone on both machines:

```bash
git clone https://github.com/ADHU-ADHU/roverlink-ros2.git
cd roverlink-ros2
```

Set these in every terminal on both machines, replacing `humble` only for a deliberately ported setup:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

The network must allow the selected DDS implementation to discover the other machine. Check `/robot/pose` and `/scan` from the PC before starting a command publisher. Device permissions, wheel polarity and the robot's physical stop method are hardware setup requirements.

## Build the PC workspace

From the repository root:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pygame python3-numpy python3-matplotlib ros-humble-slam-toolbox ros-humble-rviz2
cd pc/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If rosdep is not initialized, follow your ROS installation's rosdep setup first. Source this workspace in each PC terminal. Joystick controls require an attached pygame-compatible controller and a graphical session.

## Build and start the robot side

Follow the [robot setup](../robot/README.md) for I²C dependencies, both custom packages, and the pinned external LiDAR driver. The recorded startup is three separate robot terminals, after sourcing the matching workspace:

```bash
# Terminal R1: motor command receiver
ros2 run md25_simple md25_controller --ros-args -p speed_limit_steps:=35
```

```bash
# Terminal R2: pose and IMU
ros2 launch mpu6050_ros2 mpu6050.launch.py
```

```bash
# Terminal R3: LiDAR and sensor transform, after setting ROVERLINK_ROOT
# and sourcing robot/ldlidar_ws/install/setup.bash as in the robot guide
ros2 launch "$ROVERLINK_ROOT/dependencies/ldlidar/ld06.launch.py"
```

The reference serial path is `/dev/ttyAMA0`; adjust only to match actual wiring. The recorded driver command limits each wheel command to ±35 raw steps around neutral 128. This is a command cap, not a calibrated speed in metres per second.

## PC mapping and visualization

In separate sourced PC terminals:

```bash
ros2 run robot_slam_bringup pose2d_to_tf
```

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:="$(ros2 pkg prefix --share robot_slam_bringup)/config/slam_params.yaml"
```

```bash
rviz2
```

In RViz2 choose `map` as the fixed frame and add TF, `/map`, `/scan`, and optionally `/planned_path`. The map resolution is 0.05 m. The planner uses the robot pose directly, so check the map/odometry alignment described in [architecture](architecture.md).

## Mode A · Manual driving and recorded-route return

Do not run the frontier path follower in this mode. Start the return controller **before** teaching the route so it records the complete pose history. Use a dedicated writable directory for its JSON outputs:

```bash
mkdir -p ~/roverlink-runtime
cd ~/roverlink-runtime
ros2 run roverlink_control return_controller --ros-args -p avoid_turn_sign:=-1.0
```

Then start the joystick on the PC:

```bash
ros2 run roverlink_control teleop_joystick
```

| Input | Action |
| --- | --- |
| Left stick vertical, axis 1 | Forward/reverse |
| Right stick horizontal, axis 3 | Turn |
| LB, button 4, first press | Publish return trigger, send a brief zero-command burst and pause manual commands |

Axis mapping is controller-dependent. The teaching run assumes its start is pose `(0,0)`. The return controller reverses and subsamples the recorded path and appends that origin.

**Returning to manual:** stop `return_controller` first, then press LB again to resume manual publishing. A second LB press alone does not cancel autonomous return. No command multiplexer enforces this handover; two active motion publishers will contend. After a completed return, restart the return controller for a fresh taught route.

To stop this mode, stop the command publishers and confirm zero motor output. The responsive motor driver has a 0.2 s command-loss watchdog. It does not cover an upstream node that continues publishing commands from stale sensor data.

## Mode B · Frontier exploration

Stop both joystick and return controller. Keep the robot services, pose bridge and SLAM running. Start these in three PC terminals:

```bash
ros2 run roverlink_control frontier_selector
```

```bash
ros2 run roverlink_control astar_planner
```

```bash
ros2 run roverlink_control path_follower
```

The original recording names map to these executables:

| Recorded script | Published executable |
| --- | --- |
| `frontier_explorer55.py` | `frontier_selector` |
| `a*_smooth5.py` | `astar_planner` |
| `path_follower5.py` | `path_follower` |

The selector's plot window uses Matplotlib, so this workflow expects a graphical PC session. Treat frontier exploration as a separate experimental mode, not an automatic stage of the return demonstration.

## Check the graph

```bash
ros2 topic list -t
ros2 topic info /cmd_vel --verbose
ros2 topic info /frontier_goal --verbose
ros2 topic hz /robot/pose
ros2 topic hz /scan
```

Before motion, `/cmd_vel` should have only the intended active command source. `/frontier_goal` is `geometry_msgs/msg/PoseStamped`. Inspect the source and [known limitations](validation.md) when changing tuning or attaching other controllers.
