#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import pigpio
import math
from threading import Lock

class VehicleDriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver_node')
        
        # ========== CONFIGURATION ==========
        self.RPWM, self.LPWM = 13, 12
        self.R_EN, self.L_EN = 6, 5
        self.PWM_FREQ = 1000
        self.SERVO_PIN = 18
        self.NEUTRAL_US, self.MIN_US, self.MAX_US = 1500, 1000, 2000
        self.ENCODER_A, self.ENCODER_B = 17, 27   
        
        # Parameters
        self.declare_parameter('wheel_base', 0.135)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('max_steering_angle', 35.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('encoder_ticks_per_rev', 469)
        self.declare_parameter('speed_scaling', 1.0)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = math.radians(self.get_parameter('max_steering_angle').value)
        self.max_speed = self.get_parameter('max_speed').value
        self.encoder_ticks = self.get_parameter('encoder_ticks_per_rev').value
        self.speed_scaling = self.get_parameter('speed_scaling').value
        
        # State
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.current_steering_angle = 0.0  # <--- NEW: Track steering
        self.last_time = self.get_clock().now()
        self.rear_ticks = 0 
        self.odom_lock = Lock()
        
        # GPIO
        self.pi = pigpio.pi()
        if not self.pi.connected: return
        self.pi.set_mode(self.RPWM, pigpio.OUTPUT)
        self.pi.set_mode(self.LPWM, pigpio.OUTPUT)
        self.pi.set_mode(self.R_EN, pigpio.OUTPUT)
        self.pi.set_mode(self.L_EN, pigpio.OUTPUT)
        self.pi.write(self.R_EN, 1)
        self.pi.write(self.L_EN, 1)
        self.pi.set_PWM_frequency(self.RPWM, self.PWM_FREQ)
        self.pi.set_PWM_frequency(self.LPWM, self.PWM_FREQ)
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.NEUTRAL_US)
        
        # Encoder
        self.pi.set_mode(self.ENCODER_A, pigpio.INPUT)
        self.pi.set_mode(self.ENCODER_B, pigpio.INPUT)
        self.pi.set_pull_up_down(self.ENCODER_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.ENCODER_B, pigpio.PUD_UP)
        self.levA = self.pi.read(self.ENCODER_A)
        self.levB = self.pi.read(self.ENCODER_B)
        self.cbA = self.pi.callback(self.ENCODER_A, pigpio.EITHER_EDGE, self._encoder_callback)
        self.cbB = self.pi.callback(self.ENCODER_B, pigpio.EITHER_EDGE, self._encoder_callback)
        
        # ROS
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self) # Still good to have, though EKF handles TF usually
        self.odom_timer = self.create_timer(0.05, self.publish_odometry) # 20Hz

    def _encoder_callback(self, gpio, level, tick):
        if level > 1: return
        if gpio == self.ENCODER_A: self.levA = level
        else:
            self.levB = level
            return
        
        with self.odom_lock:
            if self.levA == self.levB: self.rear_ticks -= 1 
            else: self.rear_ticks += 1

    def cmd_vel_callback(self, msg: Twist):
        # 1. Store desired steering angle
        if abs(msg.linear.x) < 0.01:
            steering_angle = 0.0
        else:
            desired = math.atan2(msg.angular.z * self.wheel_base, msg.linear.x)
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, desired))
        
        self.current_steering_angle = steering_angle # <--- NEW: Save for Odometry calc
        
        # 2. Drive Motors
        speed_cmd = (msg.linear.x / self.max_speed) * self.speed_scaling
        duty = int(abs(speed_cmd) * 1000000)
        duty = min(max(duty, 0), 1000000)
        
        if speed_cmd > 0:
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, duty)
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, 0)
        elif speed_cmd < 0:
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, duty)
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, 0)
        else:
            self.pi.hardware_PWM(self.RPWM, 0, 0)
            self.pi.hardware_PWM(self.LPWM, 0, 0)

        # 3. Drive Servo
        angle_deg = math.degrees(steering_angle)
        pulse = self.NEUTRAL_US - (angle_deg / math.degrees(self.max_steering_angle)) * (self.MAX_US - self.NEUTRAL_US)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, int(pulse))

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        with self.odom_lock:
            ticks = self.rear_ticks
            self.rear_ticks = 0

        # Calculate Distance
        dist = ticks * ((2.0 * math.pi * self.wheel_radius) / self.encoder_ticks)
        
        # Calculate Rotation (Ackermann Kinematics)
        # delta_theta = (distance / wheelbase) * tan(steering_angle)
        if abs(self.current_steering_angle) > 0.001:
            d_theta = (dist / self.wheel_base) * math.tan(self.current_steering_angle)
        else:
            d_theta = 0.0

        # Update Pose
        self.x += dist * math.cos(self.theta)
        self.y += dist * math.sin(self.theta)
        self.theta += d_theta

        # Publish
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = dist / dt
        odom.twist.twist.angular.z = d_theta / dt

        # COVARIANCE TUNING: 
        # We are confident in X (0.01), but LESS confident in Yaw (0.2) because steering slips.
        # This helps the EKF prefer the IMU for rotation.
        odom.pose.covariance[0] = 0.01   # X
        odom.pose.covariance[7] = 0.01   # Y
        odom.pose.covariance[35] = 0.2   # Yaw (High variance = Low trust)
        
        odom.twist.covariance[0] = 0.01  # Vx
        odom.twist.covariance[35] = 0.2  # Vyaw (High variance = Low trust)

        self.odom_pub.publish(odom)
        self.last_time = now

    def destroy_node(self):
        self.pi.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = VehicleDriverNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()