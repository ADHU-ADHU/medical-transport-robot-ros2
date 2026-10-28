# Hardware and platform

The goal is to build a working mobile robot from individual hardware components and integrate its drive, sensing, communication and control through ROS 2. Hardware services run on the robot; the operator PC provides teleoperation, visualization and the selected planning/return processes.

## Selected implementation

| Component | Role in the published stack |
| --- | --- |
| Raspberry Pi 4 | Onboard ROS 2 computer connected to the motor controller and sensors |
| MD25 dual motor controller | Receives left/right motor demands over I2C bus 1 at address `0x58` |
| Two EMG30 motors with encoders | Propulsion and wheel-displacement feedback |
| LDROBOT 2D LiDAR | Laser scans for operator feedback, mapping and local obstacle handling; the supplied launch profile selects **LD06**, 230400 baud and `/dev/ttyAMA0` |
| MPU-series IMU | Gyro heading input; the published node uses the **MPU6050** interface at I2C address `0x68` |
| Operator PC and Wi-Fi network | Remote input, ROS 2 communication, SLAM/visualization and return-control coordination |

The published software targets the **LD06** and **MPU6050** interfaces. Confirm the installed sensor models, serial connection and I2C wiring before selecting drivers.

The pose node assumes a 0.10 m wheel diameter, 0.35 m wheel separation and 360 encoder ticks per revolution. These are calibration values in the code and should be checked for the actual assembly. The selected node publishes both gyro-heading/encoder position and an encoder-only comparison pose. See [robot setup](../robot/README.md) for topic units, launch commands and hardware-access requirements.

## Drive system

The platform uses **differential drive**. The left and right wheels are driven independently through the MD25 controller. Changing their relative speeds steers the robot; equal speeds produce straight travel. This arrangement does not use a mechanical differential gearbox.

## Hardware and development resources

The following resources were available for building the platform. The selected implementation above identifies the interfaces used by the published software; the list below also includes supporting equipment and components for further integration.

| Group | Provided items |
| --- | --- |
| Sensing, drive and compute | LD19 LiDAR with Raspberry GPIO mounting; MPU9250 IMU; MD25 controller; two EMG30 motors with encoders; Raspberry Pi 4 with a 16 GB SD card; TTL 3.3 V/5 V signal converter |
| Power and network | 12 V battery and charger; emergency motor-power cutoff hardware; USB power bank; USB-A power adapter; USB-C cable; access point and Ethernet cable |
| Arrival indication and payload interfaces | Buzzer; LEDs; micro switch; ULN2004 Darlington driver; PCF8574 I2C port expander; padlock with key |
| Development boards and assembly tools | ESP32 Huzzah (M65); Arduino Uno and USB cables; two breadboards; jumper cables; screwdriver; double-sided tape; cable ties and cable tape; scissors and Post-it notes |

The example equipment-delivery brief also considers arrival indication and payload handling. The curated implementation does not establish completed buzzer, flashing-light or parcel-release integration. Those components belong to the broader platform resources and future integration scope.
