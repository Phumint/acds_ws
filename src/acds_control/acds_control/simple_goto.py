import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SimpleGoToGoal(Node):
    def __init__(self):
        super().__init__('simple_goto')
        
        # --- CONFIGURATION ---
        self.goal_x = 0.0
        self.goal_y = -1.0
        self.goal_tolerance = 0.30 
        
        # --- TUNING PARAMETERS ---
        self.max_speed = 0.25       # Cruise speed (m/s)
        self.min_speed = 0.12       # Floor speed (prevent stalling when going straight)
        
        # CORNERING BOOST (The Fix)
        # Adds extra speed when turning to overcome friction.
        # Formula: speed += abs(heading_error) * corner_boost_gain
        # Example: If error is 1.0 rad (~57 deg) and gain is 0.1, adds 0.1 m/s to speed.
        self.corner_boost_gain = 0.15 
        
        self.steer_gain = 1.2       # Steering aggression
        self.look_ahead_dist = 1.0  # Distance to start slowing down
        
        # State
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.running = True

        # Communication
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Navigating to Goal: [{self.goal_x}, {self.goal_y}]")

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_heading_error(self, target_angle):
        error = target_angle - self.curr_yaw
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi
        return error

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

    def control_loop(self):
        if not self.running:
            return

        # 1. Calculate Distance & Angle
        dist = math.sqrt((self.goal_x - self.curr_x)**2 + (self.goal_y - self.curr_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.curr_y, self.goal_x - self.curr_x)
        heading_error = self.get_heading_error(angle_to_goal)

        # 2. Check Goal Reached
        if dist < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info(f"GOAL REACHED! Final Pos: [{self.curr_x:.2f}, {self.curr_y:.2f}]")
            self.running = False
            return

        msg = Twist()

        # 3. Steering Command
        steer_cmd = heading_error * self.steer_gain
        msg.angular.z = steer_cmd

        # 4. Speed Calculation with Cornering Boost
        
        # A. Base Speed (Distance based)
        # Slow down as we approach the goal
        if dist < self.look_ahead_dist:
            base_speed = self.max_speed * (dist / self.look_ahead_dist)
        else:
            base_speed = self.max_speed

        # B. Apply Floor (Minimum speed)
        base_speed = max(base_speed, self.min_speed)

        # C. Calculate Cornering Boost
        # The sharper the turn (heading_error), the more speed we add.
        # We cap the error at 1.0 radian (~57 deg) so we don't boost infinitely on crazy errors.
        turn_intensity = min(abs(heading_error), 1.0)
        boost = turn_intensity * self.corner_boost_gain
        
        # D. Final Speed Sum
        final_speed = base_speed + boost
        
        # Safety cap (optional, to prevent running too wild)
        final_speed = min(final_speed, self.max_speed + 0.2) 

        msg.linear.x = final_speed
        self.cmd_pub.publish(msg)

        # --- DEBUG OUTPUT ---
        self.get_logger().info(
            f"Pos: [{self.curr_x:.2f}, {self.curr_y:.2f}] | "
            f"Goal: [{self.goal_x:.2f}, {self.goal_y:.2f}] | "
            f"Err: {heading_error:.2f} | "
            f"BaseSpd: {base_speed:.2f} + Boost: {boost:.2f} = {final_speed:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoToGoal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        node.stop_robot()
        time.sleep(0.1)
        node.stop_robot()
        time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()