# Source selection and attribution

This repository is curated from the author's supplied PC/robot archives and additional working workspaces. The personal reflection was used to identify the author's contributions. A public copy of the [personal reflection](reports/reflection-report.pdf) is included at the author’s request, with the cover university login and two external hyperlinks removed.

## Selected implementation

| Published component | Original source |
| --- | --- |
| Joystick | Supplied PC workspace, `teleop_joy55.py` |
| Extended return | Recovered robot/control workspace, `gaussion_return555.py` |
| Frontier selector | Additional PC workspace, `frontier_explorer55.py` |
| A* planner | Additional PC workspace, `a*_smooth5.py` |
| Path follower | Additional PC workspace, `path_follower5.py` |
| SLAM adapter/config | Supplied PC workspace, `robot_slam_bringup/` |
| Motor receiver | Recovered robot/control workspace, `md25_simple/` |
| Pose/IMU | Recovered robot/control workspace, `mpu6050_ros2/` |
| Gaussian return and TSP | Recovered robot/control workspace, separate experiments |
| Heading filters | Supplied PC workspace, `Kalman filter heading.ipynb` |

Selection follows implemented behavior and recorded commands, rather than assuming a filename or timestamp establishes the final version. The exploration trio matches the filenames visible in the desktop recordings and the custom physical-robot topic contracts. This does not prove that every recording uses byte-identical source.

The [machine-readable manifest](source-manifest.json) gives original and published SHA-256 hashes and lists publication edits. Packaging, descriptive metadata and readable filenames were added. The follower's `PointStamped` subscription was corrected to match the `PoseStamped` publisher. The selected return source retains its original control logic; the documented avoidance sign is a deployment parameter override.

## External components

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) supplies mapping. This repository contains the pose adapter and selected configuration, not a new SLAM implementation. SLAM integration/tuning formed part of the wider robot effort.
- [LDROBOT LD LiDAR ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_ros2) supplies the LiDAR driver and SDK. The dependency reference is pinned, the project's serial launch adaptation is identified, and the upstream MIT notice is retained. See [robot setup](../robot/README.md).
- ROS 2, pygame, NumPy, Matplotlib, SciPy and OR-Tools are dependencies with their own licenses. SciPy and OR-Tools are needed only for the corresponding experiments.

No new blanket open-source license is assigned to the author's code. Existing upstream notices apply to their respective files. Public availability alone does not establish reuse permission.

## Excluded material

Virtual environments, build/install/log trees, caches, bag recordings, generated data, duplicate drafts and complete vendored dependency trees are omitted. Other contributors' line-following branches and the separate GPS/range-bearing estimation exercises are not presented as the author's implementation. Other personal documents and original notebook outputs are not published.

## Media

Two full physical-robot clips are included as compact, silent H.264 downloads. Paired views place the full Run 1 video beside its frontier map and the full Run 2 video beside a selected SLAM window. Each pair uses one playback clock with approximate visual alignment. The frontier disturbance capture remains separate, without a corresponding physical robot video. Individual desktop excerpts show frontier plots and RViz2. Original recordings remain unchanged outside the repository. [Media manifests and captions](demos/README.md) record the transforms and scope of what is visible; public Drive links provide browser playback of the original live runs.
