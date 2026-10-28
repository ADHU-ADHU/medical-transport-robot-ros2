from setuptools import find_packages, setup

package_name = "roverlink_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adhithya Palanisamy Chandrasekaran",
    maintainer_email="130308572+ADHU-ADHU@users.noreply.github.com",
    description="ROS 2 operator controls, teach-and-return navigation and frontier exploration.",
    license="UNLICENSED",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "teleop_joystick = roverlink_control.teleop_joystick:main",
            "return_controller = roverlink_control.return_controller:main",
            "frontier_selector = roverlink_control.frontier_selector:main",
            "astar_planner = roverlink_control.astar_planner:main",
            "path_follower = roverlink_control.path_follower:main",
        ],
    },
)
