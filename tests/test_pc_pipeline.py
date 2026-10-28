"""Synthetic ROS checks; run with a sourced PC workspace and isolated DDS domain.

These checks do not open a GUI, initialize a joystick or use robot hardware.
"""

import importlib.metadata
import math
from pathlib import Path
import time

import numpy as np
import pytest

pytest.importorskip("rclpy")

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor

from roverlink_control.astar_planner import FrontierAStarPlanner
from roverlink_control.frontier_selector import FrontierNode
from roverlink_control.path_follower import PathFollowerExplore


def test_selector_planner_follower_share_goal_message_contract(monkeypatch, tmp_path):
    """A selector goal must reach both the planner and the path follower."""
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "127")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        selector = FrontierNode()
        planner = FrontierAStarPlanner()
        follower = PathFollowerExplore()
        nodes.extend((selector, planner, follower))
        for node in nodes:
            executor.add_node(node)

        # Free room with an unexplored strip. No physical sensor publisher is used.
        grid = np.zeros((40, 40), dtype=np.int16)
        grid[30:, :] = -1
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.width = 40
        msg.info.height = 40
        msg.info.resolution = 0.1
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        pose = Pose2D(x=1.0, y=1.0, theta=90.0)
        selector.on_pose(pose)
        selector.on_map(msg)
        planner.on_pose(pose)
        planner.on_map(msg)
        follower.on_pose(pose)

        # Wait for local DDS discovery before emitting the single selector goal.
        deadline = time.monotonic() + 5.0
        while selector.goal_pub.get_subscription_count() < 2:
            executor.spin_once(timeout_sec=0.05)
            assert time.monotonic() < deadline, "Both PoseStamped goal subscribers must connect"

        selector.maybe_publish_next_goal()
        while follower.current_frontier_goal is None or not follower.have_path:
            executor.spin_once(timeout_sec=0.05)
            assert time.monotonic() < deadline, "Goal and planned path did not reach the follower"

        assert planner.have_goal
        assert planner.failed_plan_count == 0
        assert len(follower.waypoints) >= 2
        assert follower.current_frontier_goal == pytest.approx(selector.current_goal_world)
        assert follower.theta == pytest.approx(math.pi / 2.0)
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_installed_entrypoints_and_slam_configuration():
    """The built overlay exposes all nodes and installs the SLAM parameter file."""
    distribution = importlib.metadata.distribution("roverlink_control")
    expected = {
        "teleop_joystick", "return_controller", "frontier_selector",
        "astar_planner", "path_follower",
    }
    scripts = {entry.name for entry in distribution.entry_points if entry.group == "console_scripts"}
    assert expected <= scripts
    prefix = Path(get_package_prefix("roverlink_control"))
    for name in expected:
        assert (prefix / "lib" / "roverlink_control" / name).is_file()

    slam_share = Path(get_package_share_directory("robot_slam_bringup"))
    config = slam_share / "config" / "slam_params.yaml"
    assert config.is_file()
    import yaml
    params = yaml.safe_load(config.read_text())["slam_toolbox"]["ros__parameters"]
    assert params["scan_topic"] == "/scan"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_link"
    assert params["map_frame"] == "map"
