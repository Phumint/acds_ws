import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

SERVO_PIN = 18
NEUTRAL_US = 1500
MIN_US = 1000
MAX_US = 2000
MAX_WHEEL_ANGLE = 20.0  # degrees

class SteeringDriverNode(Node):
    def __init__(self):
        super().__init__('steering_driver_node')
        self.subscription_angle = self.create_subscription(Float32, 'steering_angle', self.angle_callback, 10)
        
        self.pi = pigpio.pi()
        self.pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(SERVO_PIN, NEUTRAL_US)

    def angle_callback(self, angle_deg: Float32):
        angle_deg = max(-MAX_WHEEL_ANGLE, min(MAX_WHEEL_ANGLE, angle_deg.data))
        pulse = NEUTRAL_US + (angle_deg/MAX_WHEEL_ANGLE) * (MAX_US - NEUTRAL_US)

        self.pi.set_servo_pulsewidth(SERVO_PIN, pulse)

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
        steering_driver_node.pi.set_servo_pulsewidth(SERVO_PIN, 0)
        steering_driver_node.pi.stop()

if __name__ == '__main__':
    main()