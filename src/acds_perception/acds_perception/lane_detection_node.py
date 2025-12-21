import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import os

# ==============================================================================
#  ALGORITHM CLASS (HSV + Canny + Contour Sliding Window)
# ==============================================================================

class LaneDetectionAlgorithm:
    def __init__(self):
        # --- HARDCODED PARAMETERS (From your Trackbar Defaults) ---
        # Perspective Points
        self.bl, self.tl, self.tr, self.br = (30, 477), (115, 247), (520, 255), (638, 473)
        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])    
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        # Tuning Parameters
        self.lower_white = np.array([43, 32, 80])      # H:0, S:0, V:200
        self.upper_white = np.array([179, 255, 255])   # H:179, S:60, V:255
        self.canny_low = 55
        self.canny_high = 174
        self.color_weight = 0.7  # 7 / 10
        self.final_thresh = 50
        
        # Smoothing variables
        self.prevLx, self.prevRx = [], []
        self.alpha = 0.5

    def process_frame(self, frame):
        height, width = frame.shape[:2]
        
        # 1. PRE-PROCESSING
        bird_eye = cv2.warpPerspective(frame, self.matrix, (640, 480))
        gray = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2HSV)

        # Noise Reduction
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)

        # 2. FEATURE EXTRACTION
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        canny_edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Weighted Blending
        edges_dilated = cv2.dilate(canny_edges, np.ones((3,3), np.uint8), iterations=1)
        edge_weight = 1.0 - self.color_weight
        
        combined_prob = cv2.addWeighted(white_mask, self.color_weight, edges_dilated, edge_weight, 0)
        
        # Binary Threshold
        _, mask = cv2.threshold(combined_prob, self.final_thresh, 255, cv2.THRESH_BINARY)
        
        # Morphological Clean
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 3. SLIDING WINDOW (CONTOUR BASED)
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0]//2)
        
        # Safe argmax: if image is all black, default to quarters
        if np.max(histogram[:midpoint]) > 0:
            left_base = np.argmax(histogram[:midpoint])
        else:
            left_base = width // 4

        if np.max(histogram[midpoint:]) > 0:
            right_base = np.argmax(histogram[midpoint:]) + midpoint
        else:
            right_base = 3 * width // 4

        y = mask.shape[0]
        lx, rx = [], []
        
        # We need to track Y coordinates for Heading Calculation later
        ly_coords, ry_coords = [], [] 

        while y > 0:
            # Left Window
            win_l = mask[max(0, y-20):y, max(0, left_base-50):min(640, left_base+50)]
            cnts_l, _ = cv2.findContours(win_l, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts_l:
                c = max(cnts_l, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    left_base = left_base - 50 + int(M["m10"] / M["m00"])
                    lx.append(left_base)
                    ly_coords.append(y - 10) # approximate center of window

            # Right Window
            win_r = mask[max(0, y-20):y, max(0, right_base-50):min(640, right_base+50)]
            cnts_r, _ = cv2.findContours(win_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts_r:
                c = max(cnts_r, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    right_base = right_base - 50 + int(M["m10"] / M["m00"])
                    rx.append(right_base)
                    ry_coords.append(y - 10)

            y -= 20

        # 4. TEMPORAL SMOOTHING
        if self.prevLx and len(lx) == len(self.prevLx):
            lx = (np.array(lx) * self.alpha + np.array(self.prevLx) * (1 - self.alpha)).astype(int).tolist()
        self.prevLx = lx
        
        if self.prevRx and len(rx) == len(self.prevRx):
            rx = (np.array(rx) * self.alpha + np.array(self.prevRx) * (1 - self.alpha)).astype(int).tolist()
        self.prevRx = rx

        # --------------------------------------------------------
        # CALCULATE OUTPUTS (Offset & Heading)
        # --------------------------------------------------------
        
        # Default values (used if no lanes are found)
        offset_norm = 0.0
        heading_rad = 0.0

        # Only calculate if we have enough points to be confident
        if len(lx) > 2 and len(rx) > 2:
            # 1. OFFSET: Deviation of lane center from image center
            # We use the bottom-most points (lx[0], rx[0] are approx y=480)
            lane_center_px = (lx[0] + rx[0]) / 2
            offset_norm = (lane_center_px - (width / 2)) / (width / 2)

            # 2. HEADING: Angle of the lane lines
            # Fit lines to the detected points (x = my + c). 
            # Slope 'm' (dx/dy) indicates angle relative to vertical.
            slopes = []
            fit_l = np.polyfit(ly_coords[:len(lx)], lx, 1)
            slopes.append(fit_l[0]) # slope dx/dy
            
            fit_r = np.polyfit(ry_coords[:len(rx)], rx, 1)
            slopes.append(fit_r[0])
            
            avg_slope_inv = np.mean(slopes) # dx/dy
            # heading = arctan(-dx/dy). 
            # Negative sign because Y increases downwards in image, 
            # but we want positive Y to be "forward" in robot frame.
            heading_rad = float(np.arctan(-avg_slope_inv))
        
        # 5. VISUALIZATION
        overlay = bird_eye.copy()
        if len(lx) > 0 and len(rx) > 0:
            pts_left = np.array([[x, 480 - i*20] for i, x in enumerate(lx)])
            # Note: rx might be different length than lx, fillPoly just needs perimeter points.
            pts_right = np.array([[x, 480 - i*20] for i, x in enumerate(rx)])
            
            # Create polygon
            pts = np.vstack([pts_left, np.flipud(pts_right)])
            cv2.fillPoly(overlay, [np.int32(pts)], (0, 255, 0))

        # Unwarp
        unwarped_lane = cv2.warpPerspective(overlay, self.inv_matrix, (width, height))
        
        # Blend with original frame
        result = cv2.addWeighted(frame, 1, unwarped_lane, 0.5, 0)
        
        # Add Text Info
        cv2.putText(result, f"Off: {offset_norm:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(result, f"Head: {heading_rad:.2f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return float(offset_norm), float(heading_rad), result


# ==============================================================================
#  ROS NODE (Standard Wrapper)
# ==============================================================================

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Publishers
        self.pub_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_heading = self.create_publisher(Float32, 'lane_heading', 10)

        # Parameters
        self.declare_parameter('record', True)
        self.declare_parameter('output_path', '/home/rppi4/workspace/acds_ws/lane_output.avi')
        # T-section parameter removed

        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value

        # Initialize the Algorithm
        self.get_logger().info(f"Initializing Hybrid Algorithm (HSV+Canny).")
        self.detector = LaneDetectionAlgorithm()

        # Camera Setup
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warning("⚠️ Camera not opened (index 0).")
        self.latest_frame = None
        self.lock = threading.Lock()

        # Video Writer
        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (640, 480))
            self.get_logger().info(f"🎥 Recording to: {self.output_path}")

        # Threading
        threading.Thread(target=self._capture_frames, daemon=True).start()
        
        # Timer (20Hz)
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
        
        if frame is None:
            return

        # CALL THE ALGORITHM
        offset, heading, visual = self.detector.process_frame(frame)

        # Publish
        self.pub_offset.publish(Float32(data=offset))
        self.pub_heading.publish(Float32(data=heading))

        # Record
        if self.record and self.out and self.out.isOpened():
            self.out.write(visual)

        # Log occasionally to avoid spamming
        # self.get_logger().info(f"Off: {offset:.3f} | Head: {heading:.3f}")

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
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