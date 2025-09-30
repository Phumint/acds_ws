import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from acds_control.pid import PID

class ControllerNode(Node):
    """
    subscribes to lane_offset and publishes steering_angle and motor_speed   
    """

    def __init__(self):
        super().__init__('controller_node')
        self.lane_offset_value = 0.0
        self.lane_heading_value = 0.0

        # Subscriptions
        self.lane_offset = self.create_subscription(Float32, 'lane_offset', self.offset_callback, 10)
        self.lane_heading = self.create_subscription(Float32, 'lane_heading', self.heading_callback, 10)
        
        # Publishers
        self.pub_steer = self.create_publisher(Float32, 'steering_angle', 10)
        self.pub_speed = self.create_publisher(Float32, 'motor_speed', 10)

        # PID TUNING
        self.pid = PID(Kp=6.0, Ki=0.0, Kd=0.2, output_limits=(-20, 20))  # TUNE THESE GANGIES

        self.base_speed = 1.0 # 100% duty max, max speed

        # Timer Loop
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def offset_callback(self, msg: Float32):
        self.lane_offset_value = msg.data
        # self.get_logger().info(f"Lane offset: {self.lane_offset}")        

    def heading_callback(self, msg: Float32):
        self.lane_heading_value = msg.data
        # self.get_logger().info(f"Lane heading: {self.lane_heading}")

    def control_loop(self):
        composite = float((20* self.lane_offset_value *0.8) + (20* self.lane_heading_value *0.2))
        steer_angle = -float(self.pid.update(composite))
        
        speed_proportion = 0.3 # proportion of speed reduction at max steering
        speed = self.base_speed * (1 - min(abs(steer_angle)/20, 1)*speed_proportion) # reduce speed when steering

        # Publish the speed and steer commands
        self.pub_speed.publish(Float32(data=speed))
        self.pub_steer.publish(Float32(data=steer_angle))

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    
    try:
        rclpy.spin(controller_node)
    
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
