# Medical transport mission and implementation coverage

## Purpose

The project scenario is an unmanned medical-equipment transport unit operated remotely from a hospital control station. An operator guides it through corridors with sensor feedback and commands an automatic return to the home/start position. Obstacles in the scenario include people, beds and other corridor traffic.

This repository presents a custom-built ROS 2 prototype and the author's control and integration work. The application is medical transport; mapping and frontier exploration support or extend that work rather than defining the whole project.

## Mission requirements and repository coverage

| Mission element | Published implementation and scope |
| --- | --- |
| Remote operation with feedback | PC joystick commands over ROS 2/Wi-Fi; LiDAR scans, pose and distance topics; SLAM/RViz2 feedback. |
| Commanded return to base | Records the driven pose history, reverses and subsamples it, then tracks back toward the fixed starting origin. |
| Feedback motion control | MD25 motor receiver and waypoint/return controllers; local LiDAR avoidance attempts to rejoin the route. The published return path uses sampled pose history. |
| Pose and heading estimation | Encoder displacement with integrated gyro heading, plus an encoder-only comparison pose. Kalman-filter heading/bias variants are separate experiments, not a fused estimator wired into the main controller. |
| Manual override | The brief requests an operator override. The supplied teleop pauses when triggering return, but resuming manual publishing does not cancel return. Stop the return controller before manual takeover, as described in setup. |
| Arrival signaling and parcel release | The mission scenario includes a buzzer/light and parcel release. These are not implemented by the selected published motion/control modules. The supplied resource list contains related signaling and switch components. |
| Simulator and benchmark evaluation | A simulator and repeated corridor benchmarks belong to the wider system brief. This repository does not claim a completed simulator, quantitative benchmark campaign or clinical deployment. |

## Additional work

The author's reflection and working source add frontier selection, A* planning, path smoothing, local avoidance/rejoin tuning, heading-fusion experiments and an OR-Tools multi-stop routing prototype. These are presented in the implementation and experiment sections without treating them as the original transport mission or assuming every experiment was integrated into the recorded system.

The separate [TurtleBot3 hybrid exploration repository](https://github.com/ADHU-ADHU/turtlebot3-hybrid-exploration-ros2) provides the dedicated ROS 2 Jazzy exploration project. The custom medical transport platform here was developed with Humble; the recorded publication checks use Jazzy.

## System roles

The wider brief separates the physical vehicle, teleoperator, controller, estimator and simulator. The published deployment places user input, return/exploration control, SLAM and visualization on the PC, with motors and sensor interfaces on the robot. The simulator is not started by this workflow. See [architecture](architecture.md) and [hardware/resources](hardware.md).

## Reference context

These documents were supplied to clarify the project and equipment; they are not republished here:

- **2025Telerobotics_Projektdirective20251104.pdf:** page 1 describes medical transport and commanded return; pages 2–3 specify teleoperator, controller, estimator and physical-vehicle requirements; page 8 describes staged corridor benchmark areas. Its numbered doorways define test boundaries, not implemented delivery destinations.
- **2025Telerobotics_ProjectDiscussions.pdf:** slide 18 summarizes teleoperation, estimation, sensor feedback and benchmark work within the wider project process.
- **2025Telerobotics_Lab4.pdf:** pages 1–2 cover direct/shared control, dead reckoning and telecommands; pages 2–3 list the provided hardware and bench resources.

The [personal reflection report](reports/reflection-report.pdf) and source manifest document the author's implementation context. Requirements in the reference material are not treated as proof that a feature was completed.
