# Validation and current limits

## Publication checks

The source was reviewed together with the author's reflection, wider hardware report and recorded startup commands. The repository preserves the selected control algorithms; publication edits are listed in the [source manifest](source-manifest.json).

- The two PC ROS packages build against the available ROS 2 Jazzy installation.
- The two robot hardware packages also build against Jazzy; motor mixing was checked with mock writes, without importing or starting hardware nodes. The external LiDAR driver was identified and pinned, but a fresh driver build was not repeated.
- Two synthetic checks pass: actual local DDS delivery of `PoseStamped` goals and planned paths through selector → planner → follower, and installed executable/configuration discovery.
- The synthetic graph uses an isolated localhost ROS domain, a generated occupancy grid and no physical sensors or joystick. It does not open the plotting windows or execute the control loop on a robot.
- Python syntax, ROS package XML, notebook structure, source hashes, documentation links and prepared media are checked separately.
- Both published full MP4s decode completely. Their playback speed and framing are retained; audio is removed. Preview transforms are recorded in the media manifest.

No live robot, motor, I²C device or joystick was operated during repository preparation. The original platform was Humble; a successful Jazzy package build is not a hardware compatibility result. The local test environment does not have pygame installed, so joystick execution was not tested.

To repeat the PC checks after building and sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source pc/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q tests/test_pc_pipeline.py
```

Use the matching ROS distribution for your overlay. The checks require pytest, NumPy, PyYAML and the built packages. The two package builds and synthetic test results are reproducibility checks, not measured navigation performance.

## Control and integration limits

| Area | Preserved behavior and implication |
| --- | --- |
| Command ownership | No command multiplexer exists. Stop the return controller before resuming manual control, and stop manual/return control before frontier tracking. |
| Sensor freshness | Return control warns and disables avoidance when scans are missing/stale, but can continue following. It does not check pose age. A stale-data publisher can keep sending commands and defeat the driver's command-loss watchdog. |
| Motor stopping | The responsive driver checks a 0.2 s command-loss timeout and sends zero on normal shutdown. It disables the MD25 hardware timeout. Its software watchdog is not a guarantee against a frozen driver process. |
| Local avoidance | Cooldown and committed-forward windows suppress immediate avoidance retriggers. Visibility checks do not test a swept robot footprint, and one rejoin advance is unchecked. Use this as an experimental controller, not a collision-free guarantee. |
| Return path | Reverse/subsampled history plus fixed `(0,0)` assumes teaching started at that origin. The selected main implementation has no Gaussian filter. |
| Command scale | Twist fields are normalized. `max_wheel_speed_mps` is a controller scaling estimate, not measured speed. The return source reads `max_forward`/`max_turn` but does not enforce them as independent command caps. |
| Mapping | Planning/tracking directly use Pose2D coordinates without the SLAM `map → odom` transform. Map corrections can misalign them. Moving-average smoothing is not followed by a full collision validation. |
| Estimation | The main pose node uses integrated gyro heading and encoder displacement. The Kalman filters remain separate notebooks. Raw Imu headers/orientation are incomplete; an encoder jump beyond the rejection threshold can persist without resynchronization. |
| Calibration | Wheel dimensions, scan alignment, motor polarity and avoidance turn sign belong to the recorded hardware. The IMU initializes its timing before calibration, so the first integration interval can include calibration time. |
| Multi-stop routing | The preserved TSP source has known distance-quantization and last-visit issues. It is not part of the demonstrated control chain. |

## Evidence scope

The physical clips demonstrate robot motion in an indoor test area; they do not expose the active controller. The desktop recordings show SLAM and the exploration script names. The reflection explains the author's implemented features, including work beyond the wider report. These sources support the project narrative, but do not establish repeatability statistics, collision-free operation or a completed autonomous service deployment. No such performance claim is made here.
