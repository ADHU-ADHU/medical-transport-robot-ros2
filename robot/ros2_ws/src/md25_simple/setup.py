from setuptools import setup

package_name = 'md25_simple'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adhithya Palanisamy Chandrasekaran',
    maintainer_email='130308572+ADHU-ADHU@users.noreply.github.com',
    description='Normalized ROS 2 motor command receiver for the MD25 two-motor controller.',
    license='UNLICENSED',
    entry_points={
        'console_scripts': ['md25_controller = md25_simple.md25_controller:main'],
    },
)
