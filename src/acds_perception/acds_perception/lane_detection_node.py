import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import os
import math

from acds_perception.inverse_perspective import inversePerspectiveTransform
from acds_perception.searchBox import SearchBox
from acds_perception.edge import detect_edges
from acds_perception.steering import SteeringController

# ==============================================================================
#  ALGORITHM CLASS WITH SANITY CHECKS
# ==============================================================================

class LaneDetectionAlgorithm:
    def __init__(self, points_path="/home/rppi4/workspace/acds_ws/src/acds_perception/acds_perception/_point_.npz", target_size=(720, 480)):
        self.target_size = target_size
        self.w, self.h = target_size

        # 1. Load Perspective Points
        try:
            points_data = np.load(points_path)
            points = points_data["points"]
            # Order: TL, TR, BR, BL
            self.src_points = np.float32([points[2], points[3], points[1], points[0]])
            self.dst_points = np.float32([points[6], points[7], points[5], points[4]])

        except Exception as e:
            print(f"Error loading points: {e}")
            # Fallback to empty/default if file is missing
            self.src_points = self.dst_points = None

        # 2. Initialize Sub-modules
        # We initialize with a dummy frame and update later to keep objects persistent
        self.ipt = inversePerspectiveTransform(np.zeros((self.h, self.w, 3), dtype=np.uint8))
        self.steering = SteeringController(
            frame_width=self.w, 
            frame_height=self.h, 
            lookahead_distance=0.6
        )
        self.steering.set_gains(kp=1.5, ki=0.025, kd=0.8)
        
        # SearchBox is initialized as None; it will be built on the first frame
        self.search_box = None
        

    def process_frame(self, frame, debug=False):
        """
        Processes a single frame and returns the calculated steering angle.
        """
        # Resize to maintain consistency with calibrated points
        frame = cv2.resize(frame, self.target_size)

        # 1. Birdeye View Transformation
        self.ipt.frame = frame
        birdeye_view = self.ipt.inverse_perspective_transform(
                                self.src_points, self.dst_points, self.w, self.h)

        # 2. Edge Detection
        detector = detect_edges(birdeye_view)
        birdeye_edges = detector.canny_edge()

        # 3. Search Box logic
        if self.search_box is None:
            self.search_box = SearchBox(
                birdeye_view, birdeye_edges, 
                lx=100, rx=500, y=450, width=150, height=20, num_boxes = 15
            )
        else:
            self.search_box.frame = birdeye_view
            self.search_box.mask = birdeye_edges

        vis, llane, rlane = self.search_box.visualize()
        # Ensure 'vis' is 3-channel (BGR) before returning
        if len(vis.shape) == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        # 4. Steering Calculation
        steering_angle, lane_center = self.steering.calculate_steering_angle(llane, rlane)

        img_center = self.w // 2
        offset = None
        if lane_center is not None:
            offset = lane_center - img_center

        # 5. Apply Rich Visualization (Arrows, Text, etc.)
        vis = self._draw_overlay(vis, steering_angle, lane_center)

        # steering_angle = np.radians(steering_angle)

        if debug:
            self._show_debug_windows(frame, birdeye_edges, vis, steering_angle, lane_center)

        return float(offset), float(steering_angle), vis

    def _draw_overlay(self, vis, steering_angle, lane_center):
        """
        Helper to draw the arrows, text, and direction indicators on the 'vis' image.
        """
        # --- Draw Steering Info Text ---
        cv2.putText(vis, f'Steering: {steering_angle:.1f} deg', 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # --- Draw Direction Arrow ---
        center_x = self.w // 2
        center_y = self.h - 50
        arrow_length = 100

        # Calculate arrow endpoint
        angle_rad = np.radians(steering_angle)
        end_x = int(center_x + arrow_length * math.sin(angle_rad))
        end_y = int(center_y - arrow_length * math.cos(angle_rad))

        cv2.arrowedLine(vis, (center_x, center_y), (end_x, end_y), 
                        (0, 255, 255), 3, tipLength=0.3)

        # --- Draw Vertical Reference Lines ---
        # Lane Center (Magenta)
        if lane_center is not None:
            cv2.line(vis, (int(lane_center), 0), (int(lane_center), self.h), (255, 0, 255), 2)
        
        # Image Center (Yellow)
        cv2.line(vis, (center_x, 0), (center_x, self.h), (0, 255, 255), 1)

        # --- Draw Direction Status Text ---
        if steering_angle < -5:
            direction = "LEFT"
            color = (0, 165, 255)  # Orange
        elif steering_angle > 5:
            direction = "RIGHT"
            color = (0, 165, 255)  # Orange
        else:
            direction = "STRAIGHT"
            color = (0, 255, 0)    # Green
        
        cv2.putText(vis, direction, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return vis


# ==============================================================================
#  ROS NODE 
# ==============================================================================

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.pub_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_heading = self.create_publisher(Float32, 'lane_heading', 10)
        
        self.declare_parameter('record', True)
        self.declare_parameter('output_path', '/home/rppi4/workspace/acds_ws/lane_output.avi')
        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value

        self.get_logger().info(f"Initializing Lane Algorithm with SANITY CHECKS.")
        self.detector = LaneDetectionAlgorithm()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warning("⚠️ Camera not opened.")
        self.latest_frame = None
        self.lock = threading.Lock()

        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (720, 480))

        threading.Thread(target=self._capture_frames, daemon=True).start()
        self.create_timer(0.05, self.timer_callback)

    def _capture_frames(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                with self.lock:
                    self.latest_frame = frame

    def timer_callback(self):
        with self.lock:
            frame = self.latest_frame
        if frame is None: return

        offset, heading, visual = self.detector.process_frame(frame)

        self.pub_offset.publish(Float32(data=offset))
        self.pub_heading.publish(Float32(data=heading))

        if self.record and self.out and self.out.isOpened():
            self.out.write(visual)

    def cleanup(self):
        if self.cap: self.cap.release()
        if self.out: self.out.release()

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()