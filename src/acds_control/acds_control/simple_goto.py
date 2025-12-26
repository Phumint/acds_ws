import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SmartGoToZone(Node):
    def __init__(self):
        super().__init__('smart_goto_zone')
        
        # --- CONFIGURATION ---
        # Try changing this to 0.0, 0.0 to test the fix
        self.goal_x = -2.0
        self.goal_y = 2.0
        self.goal_tolerance = 0.16
        
        # --- SAFETY ZONE (GEOFENCE) ---
        self.zone_limit = 3.0 
        
        # --- TUNING ---
        self.max_speed = 0.25
        self.min_speed = 0.12
        self.corner_boost_gain = 0.20
        self.steer_gain = 1.2
        
        # --- STATE ---
        self.direction = 1  # 1 = Forward, -1 = Reverse
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.running = True
        
        # FIX: Flag to ensure we have real data before starting
        self.odom_received = False 

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Navigating to Goal: [{self.goal_x}, {self.goal_y}]")
        self.get_logger().info("Waiting for valid Odometry data...")

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # FIX: We now know where we are
        self.odom_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """Publishes 0 velocity. Handles errors gracefully if ROS is shutting down."""
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def control_loop(self):
        if not self.running: return

        # FIX: Do not run logic until we know where the robot is!
        if not self.odom_received:
            self.get_logger().info("Waiting for odometry...", throttle_duration_sec=2.0)
            return

        # --- 1. GEOFENCE CHECK ---
        if abs(self.curr_x) > self.zone_limit or abs(self.curr_y) > self.zone_limit:
            self.stop_robot()
            self.get_logger().error(
                f"🚨 ZONE BREACH! Pos: [{self.curr_x:.2f}, {self.curr_y:.2f}] is outside limit {self.zone_limit}m"
            )
            return

        # --- 2. Physics Calculations ---
        dx = self.goal_x - self.curr_x
        dy = self.goal_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        
        heading_error_front = self.normalize_angle(angle_to_goal - self.curr_yaw)

        # --- 3. Check Goal Reached ---
        if dist < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info(f"🏆 GOAL REACHED! Final Pos: [{self.curr_x:.2f}, {self.curr_y:.2f}]")
            self.running = False
            return

        # --- 4. GEAR SHIFTING LOGIC ---
        is_behind = abs(heading_error_front) > (math.pi / 2 + 0.2)
        is_in_front = abs(heading_error_front) < (math.pi / 2 - 0.2)

        target_direction = self.direction

        if self.direction == 1 and is_behind:
            target_direction = -1 
        elif self.direction == -1 and is_in_front:
            target_direction = 1 

        if target_direction != self.direction:
            self.stop_robot()
            self.direction = target_direction
            self.get_logger().warn(f"⚙️ Shifting Gear to {'REVERSE' if self.direction == -1 else 'FORWARD'}")
            time.sleep(0.5) 
            return 

        # --- 5. Steering Calculation ---
        if self.direction == 1:
            steering_error = heading_error_front
        else:
            steering_error = self.normalize_angle(angle_to_goal - (self.curr_yaw + math.pi))

        # --- 6. Speed & Boost Calculation ---
        msg = Twist()
        msg.angular.z = steering_error * self.steer_gain

        if dist < 1.0:
            target_speed = self.max_speed * (dist / 1.0)
        else:
            target_speed = self.max_speed
        
        target_speed = max(target_speed, self.min_speed)

        turn_intensity = min(abs(steering_error), 1.0)
        boost = turn_intensity * self.corner_boost_gain
        
        final_speed = target_speed + boost
        msg.linear.x = final_speed * self.direction
        
        self.cmd_pub.publish(msg)

        # --- 7. DEBUG OUTPUT ---
        gear_str = "FWD" if self.direction == 1 else "REV"
        self.get_logger().info(
            f"Pos: [{self.curr_x:.2f}, {self.curr_y:.2f}] | "
            f"Goal: [{self.goal_x:.2f}, {self.goal_y:.2f}] | "
            f"[{gear_str}] Dist: {dist:.2f}m | "
            f"Err: {heading_error_front:.2f} | "
            f"Speed: {msg.linear.x:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SmartGoToZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        if rclpy.ok():
            try:
                node.stop_robot()
                time.sleep(0.1)
            except Exception:
                pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()