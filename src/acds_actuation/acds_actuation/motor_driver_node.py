import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.RPWM = 13
        self.LPWM = 12
        self.R_EN, self.L_EN = 6, 5
        self.PWM_FREQ = 1000  # Hz

        self.subscription_speed = self.create_subscription(Float32, 'motor_speed', self.speed_callback, 10)

        self.pi = pigpio.pi()
        self.pi.set_mode(self.RPWM, pigpio.OUTPUT)
        self.pi.set_mode(self.LPWM, pigpio.OUTPUT)
        
        self.pi.set_mode(self.R_EN, pigpio.OUTPUT)
        self.pi.set_mode(self.L_EN, pigpio.OUTPUT)

        self.pi.write(self.R_EN, 1)
        self.pi.write(self.L_EN, 1)

        self.pi.set_PWM_frequency(self.RPWM, self.PWM_FREQ)
        self.pi.set_PWM_frequency(self.LPWM, self.PWM_FREQ)

        self.get_logger().info("Motor driver node initialized.")

    def speed_callback(self, speed: Float32): 
        duty = int(abs(speed.data * 1000000))
        duty = min(max(duty, 0), 1000000)  # Clamp between 0 and 1,000,000

        if speed.data > 0:
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, duty) 
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, 0) 
        elif speed.data < 0:
            self.pi.hardware_PWM(self.LPWM, self.PWM_FREQ, duty) 
            self.pi.hardware_PWM(self.RPWM, self.PWM_FREQ, 0) 
        else:
            self.pi.hardware_PWM(self.LPWM, 0, 0)
            self.pi.hardware_PWM(self.RPWM, 0, 0)
        # self.get_logger().info(f"Set right motor speed to {duty*100:.1f}%")

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO...")
        # Stop motor/servo outputs
        if hasattr(self, "pi"):  
            try:
                # If it's a motor node:
                self.pi.hardware_PWM(self.RPWM, 0, 0)
                self.pi.hardware_PWM(self.LPWM, 0, 0)
                self.pi.write(self.R_EN, 0)
                self.pi.write(self.L_EN, 0)

                # If it's a steering node with a servo:
                # self.pi.set_servo_pulsewidth(self.servo_pin, 0)

                self.pi.stop()
            except Exception as e:
                self.get_logger().warn(f"Error during cleanup: {e}")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

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

if __name__ == '__main__':
    main()