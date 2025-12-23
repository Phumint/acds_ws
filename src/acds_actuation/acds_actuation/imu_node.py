#!/usr/bin/env python3
"""
MPU6050 IMU Driver Node
Reads accelerometer and gyroscope data via I2C
Publishes to /imu/data with proper covariance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # MPU6050 I2C Configuration
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        
        # Scaling factors
        self.ACCEL_SCALE = 16384.0  # For ±2g range
        self.GYRO_SCALE = 131.0      # For ±250°/s range
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
            time.sleep(0.1)
            self.get_logger().info(f"MPU6050 initialized on I2C bus {self.i2c_bus}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
            raise
        
        # ROS setup
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu)
        
        # Calibration offsets (run calibration to get these)
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        
        self.get_logger().info(f"IMU node started - Publishing at {self.publish_rate} Hz")

    def read_word_2c(self, addr):
        """Read a signed 16-bit value from two registers"""
        high = self.bus.read_byte_data(self.MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, addr + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def read_imu_data(self):
        """Read all IMU sensor data"""
        try:
            # Read accelerometer (m/s²)
            accel_x = self.read_word_2c(self.ACCEL_XOUT_H) / self.ACCEL_SCALE * 9.81
            accel_y = self.read_word_2c(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE * 9.81
            accel_z = self.read_word_2c(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE * 9.81
            
            # Read gyroscope (rad/s)
            gyro_x = (self.read_word_2c(self.GYRO_XOUT_H) / self.GYRO_SCALE - self.gyro_offset_x) * (math.pi / 180.0)
            gyro_y = (self.read_word_2c(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE - self.gyro_offset_y) * (math.pi / 180.0)
            gyro_z = (self.read_word_2c(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE - self.gyro_offset_z) * (math.pi / 180.0)
            
            return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        except Exception as e:
            self.get_logger().warn(f"Error reading IMU data: {e}")
            return None

    def publish_imu(self):
        """Read and publish IMU data"""
        data = self.read_imu_data()
        if data is None:
            return
        
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = data
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        # Orientation is not provided by MPU6050 (no magnetometer)
        # Set orientation covariance to -1 to indicate it's not available
        imu_msg.orientation_covariance[0] = -1.0
        
        # Covariance matrices (tunable based on sensor datasheet)
        # Angular velocity covariance (rad²/s²)
        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02
        
        # Linear acceleration covariance (m²/s⁴)
        imu_msg.linear_acceleration_covariance[0] = 0.04
        imu_msg.linear_acceleration_covariance[4] = 0.04
        imu_msg.linear_acceleration_covariance[8] = 0.04
        
        self.imu_pub.publish(imu_msg)

    def calibrate_gyro(self, samples=1000):
        """
        Calibrate gyroscope by averaging readings while stationary
        Call this during initialization if needed
        """
        self.get_logger().info(f"Calibrating gyroscope with {samples} samples...")
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
        
        for _ in range(samples):
            gyro_x = self.read_word_2c(self.GYRO_XOUT_H) / self.GYRO_SCALE
            gyro_y = self.read_word_2c(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE
            gyro_z = self.read_word_2c(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE
            sum_x += gyro_x
            sum_y += gyro_y
            sum_z += gyro_z
            time.sleep(0.001)
        
        self.gyro_offset_x = sum_x / samples
        self.gyro_offset_y = sum_y / samples
        self.gyro_offset_z = sum_z / samples
        
        self.get_logger().info(f"Gyro offsets - X: {self.gyro_offset_x:.2f}, Y: {self.gyro_offset_y:.2f}, Z: {self.gyro_offset_z:.2f}")

    def cleanup(self):
        """Close I2C bus"""
        if hasattr(self, 'bus'):
            self.bus.close()

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    
    try:
        # Optionally calibrate on startup
        node.calibrate_gyro()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()