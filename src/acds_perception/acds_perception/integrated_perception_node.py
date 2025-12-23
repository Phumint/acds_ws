import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading
import os
import math
from cv_bridge import CvBridge
from ultralytics import YOLO

# Import your custom modules
from acds_perception.inverse_perspective import inversePerspectiveTransform
from acds_perception.searchBox import SearchBox
from acds_perception.edge import detect_edges
from acds_perception.steering import SteeringController

# ==============================================================================
#  LANE DETECTION ALGORITHM (Enhanced for Integrated View)
# ==============================================================================

class LaneDetectionAlgorithm:
    def __init__(self, points_path="/home/rppi4/workspace/acds_ws/src/acds_perception/acds_perception/_point_.npz", target_size=(640, 480)):
        self.target_size = target_size
        self.w, self.h = target_size

        # 1. Load Perspective Points
        try:
            points_data = np.load(points_path)
            points = points_data["points"]
            # Order: TL, TR, BR, BL
            self.src_points = np.float32([points[2], points[3], points[1], points[0]])
            self.dst_points = np.float32([points[6], points[7], points[5], points[4]])
            
            # Calculate Inverse Matrix for projecting BACK to road
            self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

        except Exception as e:
            print(f"Error loading points: {e}")
            self.src_points = self.dst_points = self.M_inv = None

        # 2. Initialize Sub-modules
        self.ipt = inversePerspectiveTransform(np.zeros((self.h, self.w, 3), dtype=np.uint8))
        self.steering = SteeringController(
            frame_width=self.w, 
            frame_height=self.h, 
            lookahead_distance=0.6
        )
        self.steering.set_gains(kp=1.5, ki=0.025, kd=0.8)
        self.search_box = None

    def process_frame(self, frame, debug=False):
        """
        Returns: 
            offset (float), 
            steering_angle (float), 
            vis_bird_eye (image)
        """
        # Resize
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
                lx=150, rx=350, y=450, width=120, height=20, num_boxes=15
            )
        else:
            self.search_box.frame = birdeye_view
            self.search_box.mask = birdeye_edges

        # 'vis' is the bird's eye view with boxes drawn
        vis, llane, rlane = self.search_box.visualize()
        
        # Ensure 'vis' is 3-channel (BGR)
        if len(vis.shape) == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        # 4. Steering Calculation
        steering_angle, lane_center = self.steering.calculate_steering_angle(llane, rlane)
        img_center = self.w // 2
        offset = lane_center - img_center if lane_center is not None else 0.0

        # 5. Add Overlay Info to Bird's Eye View
        vis = self._draw_overlay(vis, steering_angle, lane_center)

        return float(offset), float(steering_angle), vis

    def project_lane_back(self, original_frame, bird_eye_visual):
        """
        Projects the bird's eye visualization back onto the original frame coordinates.
        """
        if self.M_inv is None:
            return original_frame

        # Warp the bird's eye view back to the original perspective
        lane_overlay = cv2.warpPerspective(bird_eye_visual, self.M_inv, (self.w, self.h))
        
        # Create a basic mask (thresholding non-black pixels)
        gray_overlay = cv2.cvtColor(lane_overlay, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray_overlay, 10, 255, cv2.THRESH_BINARY)
        
        # Create a green canvas
        green_canvas = np.zeros_like(original_frame)
        green_canvas[:] = (0, 255, 0)
        
        # Extract green lanes
        green_lanes = cv2.bitwise_and(green_canvas, green_canvas, mask=mask)
        
        # Blend: Original + Green Lanes
        result = cv2.addWeighted(original_frame, 1.0, green_lanes, 0.5, 0)
        return result

    def _draw_overlay(self, vis, steering_angle, lane_center):
        cv2.putText(vis, f'Steering: {steering_angle:.1f} deg', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        center_x = self.w // 2
        center_y = self.h - 50
        arrow_length = 100
        angle_rad = np.radians(steering_angle)
        end_x = int(center_x + arrow_length * math.sin(angle_rad))
        end_y = int(center_y - arrow_length * math.cos(angle_rad))
        cv2.arrowedLine(vis, (center_x, center_y), (end_x, end_y), (0, 255, 255), 3, tipLength=0.3)
        if lane_center is not None:
            cv2.line(vis, (int(lane_center), 0), (int(lane_center), self.h), (255, 0, 255), 2)
        cv2.line(vis, (center_x, 0), (center_x, self.h), (0, 255, 255), 1)
        
        direction = "STRAIGHT"
        color = (0, 255, 0)
        if steering_angle < -5:
            direction = "LEFT"
            color = (0, 165, 255)
        elif steering_angle > 5:
            direction = "RIGHT"
            color = (0, 165, 255)
            
        cv2.putText(vis, direction, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return vis


# ==============================================================================
#  INTEGRATED ROS NODE
# ==============================================================================

class IntegratedPerceptionNode(Node):
    def __init__(self):
        super().__init__('integrated_perception_node')

        # --- Parameters ---
        self.declare_parameter('camera_prefix', '/camera1_HV0130315L0317')
        self.declare_parameter('model_path', '/home/rppi4/workspace/acds_ws/src/acds_perception/acds_perception/models/best.pt')
        self.declare_parameter('record', True)
        self.declare_parameter('output_path', '/home/rppi4/workspace/acds_ws/integrated_output.avi')

        prefix = self.get_parameter('camera_prefix').value
        model_path = self.get_parameter('model_path').value
        self.record = self.get_parameter('record').value
        self.output_path = self.get_parameter('output_path').value

        # --- Components ---
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_count = 0  # To throttle YOLO
        self.last_detection_boxes = [] # To persist boxes between YOLO runs

        # 1. Lane Detection Logic
        self.lane_detector = LaneDetectionAlgorithm()

        # 2. Traffic Sign Logic (YOLO)
        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None

        # --- Publishers ---
        self.pub_lane_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_lane_heading = self.create_publisher(Float32, 'lane_heading', 10)
        self.pub_sign_id = self.create_publisher(Int32, 'traffic_sign_id', 10)
        self.pub_sign_label = self.create_publisher(String, 'traffic_sign_label', 10)

        # --- Subscribers ---
        # CRITICAL FIX: Use qos_profile_sensor_data for Best Effort matching
        self.create_subscription(
            Image, 
            f'{prefix}/rgb_raw', 
            self.rgb_callback, 
            qos_profile_sensor_data
        )

        # --- Video Writer (Combined) ---
        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            # Output width = 640 (Left) + 640 (Right) = 1280
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 10.0, (1280, 480))
            self.get_logger().info(f"Video Recording enabled at {self.output_path}")

        # --- Timer Loop (Main Processing) ---
        self.create_timer(0.08, self.timer_callback) # ~12 FPS target

    def rgb_callback(self, msg):
        """Standard camera callback to update latest frame."""
        try:
            if msg.encoding == '8SC3':
                img = np.frombuffer(msg.data, dtype=np.byte).reshape(msg.height, msg.width, 3).astype(np.uint8)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            img = cv2.resize(img, (640, 480))
            with self.lock:
                self.latest_frame = img
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def timer_callback(self):
        """Main processing loop."""
        with self.lock:
            if self.latest_frame is None:
                # Log only occasionally to avoid spamming
                if self.frame_count % 20 == 0: 
                    self.get_logger().warn("Waiting for camera images...")
                    self.frame_count += 1
                return
            frame_orig = self.latest_frame.copy()

        self.frame_count += 1

        # ================================
        # STEP 1: Lane Detection (EVERY FRAME)
        # ================================
        offset, heading, vis_bird_eye = self.lane_detector.process_frame(frame_orig)

        # Create Augmented View
        frame_augmented = self.lane_detector.project_lane_back(frame_orig, vis_bird_eye)

        # Publish Lane Data
        self.pub_lane_offset.publish(Float32(data=offset))
        self.pub_lane_heading.publish(Float32(data=heading))

        # ================================
        # STEP 2: Traffic Sign Detection (THROTTLED)
        # ================================
        # Only run YOLO every 5 frames to save CPU for steering
        if self.model and (self.frame_count % 5 == 0):
            results = self.model(frame_orig, conf=0.4, imgsz=320, verbose=False)
            
            self.last_detection_boxes = [] # Reset boxes
            detections_for_pub = []

            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Save for drawing later
                    self.last_detection_boxes.append((cls_id, label, conf, (x1, y1, x2, y2)))
                    detections_for_pub.append((cls_id, label, conf))

            # Publish BEST detection (highest confidence)
            if detections_for_pub:
                best = max(detections_for_pub, key=lambda x: x[2])
                self.pub_sign_id.publish(Int32(data=best[0]))
                self.pub_sign_label.publish(String(data=best[1]))

        # ================================
        # STEP 3: Draw Overlays (PERSISTENT)
        # ================================
        # Draw the last known boxes (even if we didn't run YOLO this specific frame)
        for cls_id, label, conf, (x1, y1, x2, y2) in self.last_detection_boxes:
            cv2.rectangle(frame_augmented, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame_augmented, f"{label}", (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # ================================
        # STEP 4: Compose & Save Video
        # ================================
        if vis_bird_eye.shape != frame_augmented.shape:
             vis_bird_eye = cv2.resize(vis_bird_eye, (frame_augmented.shape[1], frame_augmented.shape[0]))

        combined_view = cv2.hconcat([vis_bird_eye, frame_augmented])

        if self.record and self.out and self.out.isOpened():
            self.out.write(combined_view)

        # Optional: Display locally
        # cv2.imshow("Integrated View", combined_view)
        # cv2.waitKey(1)

    def cleanup(self):
        if self.out:
            self.out.release()
            self.get_logger().info(f"Video saved to {self.output_path}")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedPerceptionNode()
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