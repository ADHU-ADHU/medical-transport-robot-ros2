from setuptools import setup

package_name = 'mpu6050_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mpu6050.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adhithya Palanisamy Chandrasekaran',
    maintainer_email='130308572+ADHU-ADHU@users.noreply.github.com',
    description='MPU6050 heading and encoder-based pose publishing for the medical transport robot.',
    license='MIT',
    entry_points={
        'console_scripts': ['mpu6050_node = mpu6050_ros2.mpu6050_node:main'],
    },
)
