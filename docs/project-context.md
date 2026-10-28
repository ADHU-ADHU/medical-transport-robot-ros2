# Mobile robot platform and implementation coverage

## Purpose

The main goal was to build a working mobile robot from components and connect its motors, sensors, estimation and control through ROS 2. The platform combines onboard hardware services with a remote operator PC for driving, sensor feedback, mapping and higher-level control.

This repository presents the author's control and integration work on that custom-built platform: joystick teleoperation, recorded-route return, local obstacle handling and the PC/robot software connection. Exploration, planning and estimation experiments extend those capabilities.

## Platform capabilities and repository coverage

| Capability | Published implementation and scope |
| --- | --- |
| Hardware and ROS bring-up | Onboard MD25 motor receiver, encoder/IMU pose node and external LiDAR driver configuration, connected to the PC through ROS 2. The hardware notes distinguish supplied resources from implemented interfaces. |
| Remote operation with feedback | PC joystick commands over ROS 2/Wi-Fi; LiDAR scans, pose and distance topics; SLAM/RViz2 feedback. |
| Commanded return to base | Records the driven pose history, reverses and subsamples it, then tracks back toward the fixed starting origin. |
| Feedback motion control | MD25 motor receiver and waypoint/return controllers; local LiDAR avoidance attempts to rejoin the route. The published return path uses sampled pose history. |
| Pose and heading estimation | Encoder displacement with integrated gyro heading, plus an encoder-only comparison pose. Kalman-filter heading/bias variants are separate experiments, not a fused estimator wired into the main controller. |
| Manual override | The brief requests an operator override. The supplied teleop pauses when triggering return, but resuming manual publishing does not cancel return. Stop the return controller before manual takeover, as described in setup. |
| Simulator and benchmark evaluation | A simulator and repeated corridor benchmarks belong to the wider system brief. This repository does not claim a completed simulator or quantitative benchmark campaign. |

## Additional work

The author's reflection and working source add frontier selection, A* planning, path smoothing, local avoidance/rejoin tuning, heading-fusion experiments and an OR-Tools multi-stop routing prototype. Their implementation and integration status are documented separately. Some remained standalone experiments.

The separate [TurtleBot3 hybrid exploration repository](https://github.com/ADHU-ADHU/turtlebot3-hybrid-exploration-ros2) provides the dedicated ROS 2 Jazzy exploration project. The custom mobile robot platform here was developed with Humble; the recorded publication checks use Jazzy.

## System roles

The wider brief separates the physical vehicle, teleoperator, controller, estimator and simulator. The published deployment places user input, return/exploration control, SLAM and visualization on the PC, with motors and sensor interfaces on the robot. The simulator is not started by this workflow. See [architecture](architecture.md) and [hardware/resources](hardware.md).

## Example application in the brief

Medical-equipment transport through hospital corridors is an example motivation supplied in the project brief. It illustrates how remote operation, live sensor feedback and a commanded return to the starting point could support a practical task. The platform work is presented around building and operating the robot itself.

That example also describes arrival indication with a buzzer/light and parcel release. The selected published modules do not implement those functions; related signaling and switch components appear in the supplied resource list. The prototype is not presented as a clinically deployed delivery system.

For implementation details and development experience, see the [personal reflection report](reports/reflection-report.pdf) and [source manifest](source-manifest.json).
