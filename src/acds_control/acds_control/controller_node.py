import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from acds_control.pid import PID

class ControllerNode(Node):
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

        # === PID TUNING (AGGRESSIVE) ===
        # Input is normalized (-1.0 to 1.0). Output is degrees (-20 to 20).
        # Kp=20.0 means: if we are 100% off track (1.0), steer 20 degrees (max).
        # Ki=0.05 helps fix small steady-state errors (if car drifts to one side).
        # Kd=0.5 adds a "brake" to the steering so it doesn't oscillate.
        self.pid = PID(Kp=0.8, Ki=0.0, Kd=0.5, output_limits=(-30, 30)) 

        self.base_speed = 0.7
        self.img_width = 640.0 

        # Timer Loop (20Hz is good for responsiveness)
        self.timer = self.create_timer(0.05, self.control_loop)

    def offset_callback(self, msg: Float32):
        self.lane_offset_value = msg.data      

    def heading_callback(self, msg: Float32):
        self.lane_heading_value = msg.data

    def control_loop(self):
        # 1. NORMALIZE (-1.0 to 1.0)
        norm_offset = self.lane_offset_value / (self.img_width / 2.0)
        norm_heading = self.lane_heading_value # Assuming this is already radians/small float

        # 2. CALCULATE ERROR
        # Increase heading weight slightly to anticipate turns earlier
        # Offset 0.6, Heading 0.4 gives a snappier response to curves
        # error = (60* 0.0 * norm_offset) + (30* 1.0 * norm_heading)
        
        # 3. PID UPDATE
        # Note: We removed the arbitrary "* 20.0" multiplier from inside the update
        # and instead folded it into the Kp gain above. It's cleaner math.
        # steer_angle = float(self.pid.update(error))
        steer_angle = (norm_heading)
        # 5. SPEED CONTROL
        # Slow down if steering angle is sharp (>10 degrees)
        speed_proportion = 0.7 
        speed = self.base_speed * (1 + min(abs(steer_angle)/30.0, 1) * speed_proportion)

        self.pub_speed.publish(Float32(data=speed))
        self.pub_steer.publish(Float32(data=steer_angle))

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()