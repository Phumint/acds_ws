import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

class SteeringDriverNode(Node):
    def __init__(self):
        super().__init__('steering_driver_node')
        # Define GPIO pin for servo
        self.SERVO_PIN = 18
        self.NEUTRAL_US = 1500
        self.MIN_US = 1000
        self.MAX_US = 2000
        self.MAX_WHEEL_ANGLE = 20.0  # degrees

        self.subscription_angle = self.create_subscription(Float32, 'steering_angle', self.angle_callback, 10)
        
        self.pi = pigpio.pi()
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.NEUTRAL_US)

    def angle_callback(self, angle_deg: Float32):
        angle_deg = max(-self.MAX_WHEEL_ANGLE, min(self.MAX_WHEEL_ANGLE, angle_deg.data))
        pulse = self.NEUTRAL_US + (angle_deg/self.MAX_WHEEL_ANGLE) * (self.MAX_US - self.NEUTRAL_US)

        self.pi.set_servo_pulsewidth(self.SERVO_PIN, pulse)

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO...")
        # Stop motor/servo outputs
        if hasattr(self, "pi"):  
            try:
                # If it's a motor node:
                # self.pi.hardware_PWM(self.RPWM, 0, 0)
                # self.pi.hardware_PWM(self.LPWM, 0, 0)
                # self.pi.write(self.R_EN, 0)
                # self.pi.write(self.L_EN, 0)

                # If it's a steering node with a servo:
                self.pi.set_servo_pulsewidth(self.SERVO_PIN, 0)

                self.pi.stop()
            except Exception as e:
                self.get_logger().warn(f"Error during cleanup: {e}")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    steering_driver_node = SteeringDriverNode()
    
    try:
        rclpy.spin(steering_driver_node)
    
    except KeyboardInterrupt:
        pass
    finally:
        steering_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()