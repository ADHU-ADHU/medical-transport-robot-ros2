from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_slam_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adhithya Palanisamy Chandrasekaran',
    maintainer_email='130308572+ADHU-ADHU@users.noreply.github.com',
    description='Pose2D-to-TF bridge and SLAM Toolbox mapping configuration.',
    license='UNLICENSED',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose2d_to_tf = robot_slam_bringup.pose2d_to_tf:main',
        ],
    },
)
