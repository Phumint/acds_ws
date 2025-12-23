#!/usr/bin/env python3
"""
Unified Vehicle Driver Node for Ackermann Robot
Subscribes to /cmd_vel and controls motor + servo
Publishes wheel odometry from encoders to /odom_raw
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import pigpio
import math
from threading import Lock

class VehicleDriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver_node')
        
        # ========== HARDWARE CONFIGURATION ==========
        # Motor Pins
        self.RPWM = 13
        self.LPWM = 12
        self.R_EN = 6
        self.L_EN = 5
        self.PWM_FREQ = 1000
        
        # Servo Pin
        self.SERVO_PIN = 18
        self.NEUTRAL_US = 1500
        self.MIN_US = 1000
        self.MAX_US = 2000
        
        # Quadrature Encoder Pins (UPDATE THESE TO YOUR ACTUAL PINS!)
        # Single encoder for rear motor (drives both rear wheels)
        self.ENCODER_A = 17   # Rear motor encoder Phase A
        self.ENCODER_B = 27   # Rear motor encoder Phase B
        
        # ========== VEHICLE PARAMETERS ==========
        self.declare_parameter('wheel_base', 0.135)  # Distance between front and rear axles (m)
        self.declare_parameter('wheel_radius', 0.033)  # Wheel radius (m)
        self.declare_parameter('max_steering_angle', 35.0)  # Max steering angle (degrees)
        self.declare_parameter('max_speed', 1.0)  # Max linear speed (m/s)
        self.declare_parameter('encoder_ticks_per_rev', 488)  # 977, Encoder resolution
        self.declare_parameter('speed_scaling', 0.9)  # Motor power scaling factor
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = math.radians(self.get_parameter('max_steering_angle').value)
        self.max_speed = self.get_parameter('max_speed').value
        self.encoder_ticks = self.get_parameter('encoder_ticks_per_rev').value
        self.speed_scaling = self.get_parameter('speed_scaling').value
        
        # ========== ODOMETRY STATE ==========
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.rear_ticks = 0  # Single encoder for rear motor
        self.odom_lock = Lock()
        
        # Quadrature encoder state tracking
        self.last_a = 0
        self.last_b = 0
        
        # ========== GPIO INITIALIZATION ==========
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon!")
            return
            
        # Motor setup
        self.pi.set_mode(self.RPWM, pigpio.OUTPUT)
        self.pi.set_mode(self.LPWM, pigpio.OUTPUT)
        self.pi.set_mode(self.R_EN, pigpio.OUTPUT)
        self.pi.set_mode(self.L_EN, pigpio.OUTPUT)
        self.pi.write(self.R_EN, 1)
        self.pi.write(self.L_EN, 1)
        self.pi.set_PWM_frequency(self.RPWM, self.PWM_FREQ)
        self.pi.set_PWM_frequency(self.LPWM, self.PWM_FREQ)
        
        # Servo setup
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.NEUTRAL_US)
        
        # Encoder setup (single rear encoder)
        self.pi.set_mode(self.ENCODER_A, pigpio.INPUT)
        self.pi.set_mode(self.ENCODER_B, pigpio.INPUT)
        
        self.pi.set_pull_up_down(self.ENCODER_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.ENCODER_B, pigpio.PUD_UP)
        
        # Initialize encoder states
        self.last_a = self.pi.read(self.ENCODER_A)
        self.last_b = self.pi.read(self.ENCODER_B)
        
        # Quadrature encoder callback (trigger on both edges of Phase A)
        self.pi.callback(self.ENCODER_A, pigpio.EITHER_EDGE, self._encoder_callback)
        
        # ========== ROS SETUP ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry publishing timer (50 Hz)
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)
        
        self.get_logger().info(f"Vehicle Driver initialized - Wheelbase: {self.wheel_base}m, Max Steering: {math.degrees(self.max_steering_angle)}°")

    def _encoder_callback(self, gpio, level, tick):
        """
        Quadrature encoder callback for rear motor
        Determines direction based on phase relationship between A and B
        Single encoder tracks both rear wheels (they're mechanically linked)
        """
        # Read current states
        a_state = self.pi.read(self.ENCODER_A)
        b_state = self.pi.read(self.ENCODER_B)
        
        # Determine direction using quadrature encoding
        # Forward:  A leads B
        # Backward: B leads A
        
        with self.odom_lock:
            if a_state == self.last_b:
                self.rear_ticks += 1  # Forward
            else:
                self.rear_ticks -= 1  # Backward
            
            # Update last states
            self.last_a = a_state
            self.last_b = b_state

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist (linear.x, angular.z) to Ackermann steering + motor speed
        """
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Clamp linear velocity
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        
        # ========== ACKERMANN STEERING CONVERSION ==========
        # For Ackermann: angular_vel = (linear_vel * tan(steering_angle)) / wheelbase
        # Solve for steering_angle
        if abs(linear_vel) < 0.01:
            steering_angle = 0.0
        else:
            # Prevent division by zero in tan
            desired_angle = math.atan2(angular_vel * self.wheel_base, linear_vel)
            steering_angle = max(-self.max_steering_angle, 
                               min(self.max_steering_angle, desired_angle))
        
        # ========== MOTOR CONTROL ==========
        # Scale velocity to duty cycle (-1.0 to 1.0)
        speed_cmd = (linear_vel / self.max_speed) * self.speed_scaling
        duty = int(abs(speed_cmd) * 1000000)
        duty = min(max(duty, 0), 1000000)
        
        if speed_cmd > 0:
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, duty)
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, 0)
        elif speed_cmd < 0:
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, duty)
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, 0)
        else:
            self.pi.hardware_PWM(self.LPWM, 0, 0)
            self.pi.hardware_PWM(self.RPWM, 0, 0)
        
        # ========== SERVO CONTROL ==========
        # Map steering angle to servo pulse width
        angle_deg = math.degrees(steering_angle)
        max_angle_deg = math.degrees(self.max_steering_angle)
        pulse = self.NEUTRAL_US + (angle_deg / max_angle_deg) * (self.MAX_US - self.NEUTRAL_US)
        pulse = int(max(self.MIN_US, min(self.MAX_US, pulse)))
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, pulse)

    def publish_odometry(self):
        """
        Calculate and publish odometry from encoder ticks
        Note: Single encoder on rear motor, so left/right distances are equal
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        with self.odom_lock:
            rear_ticks = self.rear_ticks
            self.rear_ticks = 0
        
        # Convert ticks to distance (direction already encoded in tick count)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.encoder_ticks
        distance = rear_ticks * meters_per_tick
        
        # For Ackermann steering with single rear motor:
        # Both rear wheels travel the same distance (no differential)
        # Steering is done by front wheels only
        
        # Update pose (simplified - assumes small time steps)
        # Delta heading comes from steering angle (not from encoder difference)
        # For now, we track straight-line motion only
        # Full Ackermann kinematics would use steering angle feedback
        
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        # Note: theta (heading) changes are not tracked by encoders alone
        # You'd need IMU or steering angle sensor for accurate theta
        
        # Velocities
        vx = distance / dt if dt > 0 else 0.0
        vth = 0.0  # Cannot determine from single rear encoder
        # IMU will provide angular velocity in sensor fusion
        
        # ========== PUBLISH ODOMETRY MESSAGE ==========
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        # Covariance (tunable - lower = more confident)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.05  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05 # vyaw
        
        self.odom_pub.publish(odom)
        
        # ========== PUBLISH TF ==========
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time

    def cleanup(self):
        """Stop all hardware on shutdown"""
        self.get_logger().info("Shutting down vehicle driver...")
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.hardware_PWM(self.RPWM, 0, 0)
            self.pi.hardware_PWM(self.LPWM, 0, 0)
            self.pi.write(self.R_EN, 0)
            self.pi.write(self.L_EN, 0)
            self.pi.set_servo_pulsewidth(self.SERVO_PIN, 0)
            self.pi.stop()

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()