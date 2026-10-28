# Hardware and platform

The project uses a custom-built ROS 2 mobile platform for medical transport and automatic return in hospital-like indoor environments. Hardware services run on the robot; the operator PC provides teleoperation, visualization and the selected planning/return processes.

## Selected implementation

| Component | Role in the published stack |
| --- | --- |
| Raspberry Pi 4 | Onboard ROS 2 computer connected to the motor controller and sensors |
| MD25 dual motor controller | Receives left/right motor demands over I2C bus 1 at address `0x58` |
| Two EMG30 motors with encoders | Propulsion and wheel-displacement feedback; the model is identified in the supplied hardware inventory |
| LDROBOT 2D LiDAR | Laser scans for operator feedback, mapping and local obstacle handling; the supplied launch profile selects **LD06**, 230400 baud and `/dev/ttyAMA0` |
| MPU-series IMU | Gyro heading input; the published node uses the **MPU6050** interface at I2C address `0x68` |
| Operator PC and Wi-Fi network | Remote input, ROS 2 communication, SLAM/visualization and return-control coordination |

The reference material names different sensor models in different places. `2025Telerobotics_Lab4.pdf`, page 2, lists **LD19** and **MPU9250** in the equipment inventory, while its implementation tasks name **LD06** and **MPU6050**. The published configuration follows the supplied LD06 launch file and MPU6050 node. These are the software targets; the footage does not expose legible model labels that independently resolve the inventory difference.

The pose node assumes a 0.10 m wheel diameter, 0.35 m wheel separation and 360 encoder ticks per revolution. These are calibration values in the code and should be checked for the actual assembly. The selected node publishes both gyro-heading/encoder position and an encoder-only comparison pose. See [robot setup](../robot/README.md) for topic units, launch commands and hardware-access requirements.

## Provided hardware resources

The following inventory comes from `2025Telerobotics_Lab4.pdf`, pages 2-3. It records resources made available for building the platform. **Provisioning does not mean every item was integrated into the published software or demonstrated workflow.**

| Group | Provided items |
| --- | --- |
| Sensing, drive and compute | LD19 LiDAR with Raspberry GPIO mounting; MPU9250 IMU; MD25 controller; two EMG30 motors with encoders; Raspberry Pi 4 with a 16 GB SD card; TTL 3.3 V/5 V signal converter |
| Power and network | 12 V battery and charger; emergency motor-power cutoff hardware; USB power bank; USB-A power adapter; USB-C cable; access point and Ethernet cable |
| Arrival indication and payload interfaces | Buzzer; LEDs; micro switch; ULN2004 Darlington driver; PCF8574 I2C port expander; padlock with key |
| Development boards and assembly tools | ESP32 Huzzah (M65); Arduino Uno and USB cables; two breadboards; jumper cables; screwdriver; double-sided tape; cable ties and cable tape; scissors and Post-it notes |

The medical-transport concept also considers arrival indication and payload handling. The curated implementation does not establish completed buzzer, flashing-light or parcel-release integration. Those components belong to the broader platform resources and future integration scope.

## Drive-model detail

`2025Telerobotics_Projektdirective20251104.pdf`, page 3, explicitly specifies a differential-drive platform in its real-vehicle requirements. The selected controller mixes forward and turn demands into two wheel commands, and the encoder model estimates heading change from the difference between right and left wheel displacement. This is the motion model used by the supplied control and estimation code.

The public project is presented around its medical-transport mission, teleoperation and automatic-return behavior. The supplied material does not identify a commercial chassis model. The hardware laboratory establishes the control foundation; it does not describe the full scope or completion status of the medical-transport system.

The handouts named above were used as reference context and are not included in this repository.
