import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from mpu6050 import mpu6050
import time
import math
import numpy as np
from geometry_msgs.msg import Pose2D
import smbus
import math
from std_msgs.msg import Float64


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Publishers
        self.pub_raw = self.create_publisher(Imu, '/robot/imu_raw', 10) # Raw data of 6-axis, linear velcoity in m/s and angular velocity in rad/s
        self.pub_twist = self.create_publisher(Twist, '/robot/imu', 10)
        self.pub_pose = self.create_publisher(Pose2D, '/robot/pose', 10) # Publisher for pose topic,computed using wheel encoders and IMU heading 
        self.pub_pose_enc=self.create_publisher(Pose2D, '/robot/pose_enc',10)
        self.pub_distance = self.create_publisher(Float64, '/robot/distance_travelled', 10) 

        #---------------------------------------------------######
        # Position tracking variables
        self.x = 0.0  # updated using wheel encoders and IMU heading (self.heading)
        self.y = 0.0  # updated using wheel encoders and IMU heading
        #self.yaw = 0.0  # updated from IMU / gyro (not used)
        self.x_enc = 0.0 # updated using the wheel encoders only self.heading_enc)
        self.y_enc = 0.0 # updated using the wheel encoders  only (self.heading_enc)
        # Heading integration
        self.heading = 0.0  # updated from IMU / gyro , yaw
        self.heading_enc = 0.0  #  Updated from wheen encoders
        self.last_time = time.time()
        self.total_distance = 0.0 # Updated frpm wheel encoders
        #---------------------------------------------------------------------------------

        # IMU
        self.sensor = mpu6050(0x68)

        # Bias estimation
        self.gyro_bias = 0.0
        self.calibrate_bias() # for yaw

        # Linear acceleration bias 
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.calibrate_accel_bias()  # for linear acc x and y

      
        # Encoder odometry constants  and  Robot physical params
        self.wheel_diameter = 0.10  # 10 cm diameter wheel in meters
        self.wheel_base = 0.35      # wheel base / distance between wheels, 35 cm
        self.wheel_circumference = self.wheel_diameter * math.pi
        self.tick_dist = self.wheel_circumference / 360.0   # meters per encoder tick (10cm wheel)
        
        # Timer
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.get_logger().info("MPU6050 Node started")
        # I2C bus and address for encoder
        self.bus = smbus.SMBus(1)
        self.addr = 0x58  # motor controller I2C address
        
        # Reset encoder counts on startup
        try:
            self.bus.write_byte_data(self.addr, 16, 32)
            self.get_logger().info("Motor encoders reset.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset encoders: {e}")

        
        # Last encoder readings, initialising encoder reading from the hardware
        self.enc_left_last = self.read_encoder(2)
        self.enc_right_last = self.read_encoder(6)
        self.MAX_TICKS_PER_UPDATE = 40  # max allowed encoder ticks in one itteration, exceeding it classifies as anomaly
        #self.enc_left_last = 0
        #self.enc_right_last = 0


    def calibrate_bias(self):
        self.get_logger().info("Calibrating gyro bias... KEEP IMU STILL")
        samples = 500
        total = 0.0
        for _ in range(samples):
            g = self.sensor.get_gyro_data()
            total += g['z']
            time.sleep(0.002)
        self.gyro_bias = total / samples
        self.get_logger().info(f"Gyro bias = {self.gyro_bias:.6f} deg/s, samples = {samples}")

    def calibrate_accel_bias(self):
        self.get_logger().info("Calibrating accel bias... KEEP IMU STILL")
        samples = 500
        total_x, total_y = 0.0, 0.0
        for _ in range(samples):
            a = self.sensor.get_accel_data()
            total_x += a['x']
            total_y += a['y']
            time.sleep(0.002)
        self.accel_bias_x = total_x / samples
        self.accel_bias_y = total_y / samples
        #self.get_logger().info(f"Accel bias: x={self.accel_bias_x:.6f}, y={self.accel_bias_y:.6f}")
        
    def read_encoder(self, reg_start):
        data = self.bus.read_i2c_block_data(self.addr, reg_start, 4)  # [b0,b1,b2,b3]
        b0, b1, b2, b3 = data

        count = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3
        if count & 0x80000000:  # signed
            count -= 0x100000000
        return count



    def update(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        # Convert gyro to rad/s
        raw_rate = gyro['z'] * math.pi / 180.0
        corrected_rate = raw_rate - (self.gyro_bias * math.pi / 180.0)

        # Heading integration
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        self.heading += corrected_rate * dt

        # Normalise IMU heading [-pi, pi]
        if self.heading > math.pi:
            self.heading -= 2 * math.pi
        if self.heading < -math.pi:
            self.heading += 2 * math.pi
         # -----------------------------------------
         # ENCODER ODOMETRY  <-- INSERTED HERE
         # -----------------------------------------
        enc_left = self.read_encoder(2)
        enc_right = self.read_encoder(6)
        
        dL_ticks = enc_left - self.enc_left_last
        dR_ticks = enc_right - self.enc_right_last

        

        if abs(dL_ticks) > self.MAX_TICKS_PER_UPDATE:
            dL_ticks = 0
            enc_left = self.enc_left_last

        if abs(dR_ticks) > self.MAX_TICKS_PER_UPDATE:
            dR_ticks = 0
            enc_right = self.enc_right_last

        dL = dL_ticks * self.tick_dist
        dR = dR_ticks * self.tick_dist


        self.enc_left_last = enc_left
        self.enc_right_last = enc_right

        d = (dL + dR) / 2.0
        #print(d)
        theta = self.heading
        
        #self.get_logger().info(f"d = {d} , dL/dR = {dL , dR}")
        
        self.x += d * math.cos(theta)  # x updated using wheel encoder and IMU heading
        self.y += d * math.sin(theta)  # y updated using wheel encoders and IMU heading
        # -----------------------------------------
        # Heading and pose computed only with wheel encoders
        dtheta = (dR - dL)  / self.wheel_base
        theta_mid = self.heading_enc + 0.5 * dtheta # midpoint integration for better accuracy
        self.x_enc += d * math.cos(theta_mid)
        self.y_enc += d * math.sin(theta_mid)
        self.heading_enc += dtheta

        # normalize to [-pi, pi] (optional)
        if self.heading_enc > math.pi:
            self.heading_enc -= 2 * math.pi
        if self.heading_enc < -math.pi:
            self.heading_enc += 2 * math.pi

        # Total distance travelled - update
        self.total_distance += abs(d)

        # -----------------------------------------
        # Publish RAW /robot/imu_raw
        # -----------------------------------------
        msg = Imu()
        msg.angular_velocity.x = gyro['x'] * math.pi/180
        msg.angular_velocity.y = gyro['y'] * math.pi/180
        msg.angular_velocity.z = raw_rate

        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']

        self.pub_raw.publish(msg)

        # -----------------------------------------
        # Publish PROCESSED /robot/imu (Twist)
        # -----------------------------------------
        twist = Twist()
        twist.angular.x = raw_rate
        twist.angular.y = self.gyro_bias * math.pi/180
        twist.angular.z = np.mod(np.degrees(self.heading), 360)

        self.pub_twist.publish(twist)
        # -----------------------------------------
        # Publish /robot/pose
        # -----------------------------------------
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = np.mod(np.degrees(self.heading), 360)
        self.pub_pose.publish(pose)

        # Publish /robot/pose_enc
        pose_enc = Pose2D()
        pose_enc.x = self.x_enc
        pose_enc.y = self.y_enc
        pose_enc.theta = np.mod(np.degrees(self.heading_enc),360)
        self.pub_pose_enc.publish(pose_enc)


        #----- Publishing total distance travelled ----- #
        self.pub_distance.publish(Float64(data=(self.total_distance)))


        ############------------------------#
        ############------------------------#
        ############------------------------#


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()