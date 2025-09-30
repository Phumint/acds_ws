import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

RPWM = 13
LPWM = 12
R_EN, L_EN = 6, 5
PWM_FREQ = 1000  # Hz

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.subscription_speed = self.create_subscription(Float32, 'motor_speed', self.speed_callback, 10)

        self.pi = pigpio.pi()
        self.pi.set_mode(RPWM, pigpio.OUTPUT)
        self.pi.set_mode(LPWM, pigpio.OUTPUT)
        
        self.pi.set_mode(R_EN, pigpio.OUTPUT)
        self.pi.set_mode(L_EN, pigpio.OUTPUT)

        self.pi.write(R_EN, 1)
        self.pi.write(L_EN, 1)

        self.pi.set_PWM_frequency(RPWM, PWM_FREQ)
        self.pi.set_PWM_frequency(LPWM, PWM_FREQ)

        self.get_logger().info("Motor driver node initialized.")

    def speed_callback(self, speed: Float32): 
        duty = int(abs(speed.data * 1000000))
        duty = min(max(duty, 0), 1000000)  # Clamp between 0 and 1,000,000

        if speed.data > 0:
            self.pi.hardware_PWM(RPWM, PWM_FREQ, duty) 
            self.pi.hardware_PWM(LPWM, PWM_FREQ, 0) 
        elif speed.data < 0:
            self.pi.hardware_PWM(LPWM, PWM_FREQ, duty) 
            self.pi.hardware_PWM(RPWM, PWM_FREQ, 0) 
        else:
            self.pi.hardware_PWM(LPWM, 0, 0)
            self.pi.hardware_PWM(RPWM, 0, 0)
        # self.get_logger().info(f"Set right motor speed to {duty*100:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    
    try:
        rclpy.spin(motor_driver_node)
    
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.destroy_node()
        rclpy.shutdown()
        motor_driver_node.pi.hardware_PWM(RPWM, 0, 0)
        motor_driver_node.pi.hardware_PWM(LPWM, 0, 0)
        motor_driver_node.pi.write(R_EN, 0)
        motor_driver_node.pi.write(L_EN, 0)
        motor_driver_node.pi.stop()