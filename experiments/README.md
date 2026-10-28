# Control and estimation experiments

These alternatives preserve selected exploratory implementations. They are not started by the main PC/robot workflow.

| Experiment | Implemented behavior | Reproduction notes |
| --- | --- | --- |
| [`heading/heading_filter_variants.ipynb`](heading/heading_filter_variants.ipynb) | Scalar heading filter and two-state heading/gyro-bias Kalman filter, using gyro prediction and encoder heading corrections | Run one variant in a fresh kernel. Requires ROS, NumPy and live project topics. `/robot/pose_fused` is not automatically routed into SLAM or control. Notebook outputs were cleared; code cells are preserved. |
| [`return_smoothing/gaussian_return.py`](return_smoothing/gaussian_return.py) | Gaussian filtering of recorded x/y trajectories, endpoint retention and subsampled return, with a front-obstacle pause | Earlier combined hardware/controller design. Uses SciPy and writes directly to the MD25 over I²C. It must run on the robot and exclusively own the motors; do not start it with `md25_simple`. The main return controller does **not** apply Gaussian filtering. |
| [`multi_stop/tsp_prototype.py`](multi_stop/tsp_prototype.py) | Interactive waypoint collection and an OR-Tools routing prototype with direct motor control | Exploratory source, not an integrated mission. Its distance callback truncates metres to integers, and the `route[:-1]` selection drops the last visit in the optimized route. These known issues remain in the preserved prototype. Requires OR-Tools and exclusive hardware access; it was not executed during publication. |

The separate range/bearing Kalman-filter lab and unrelated GPS notebooks are not part of this robot pipeline. No accuracy figures from those exercises are used to characterize the heading filters here.
