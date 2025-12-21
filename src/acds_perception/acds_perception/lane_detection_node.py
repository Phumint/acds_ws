import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import os

# ==============================================================================
#  ALGORITHM CLASS WITH SANITY CHECKS
# ==============================================================================

class LaneDetectionAlgorithm:
    def __init__(self):
        # --- HARDCODED PARAMETERS ---
        # Bottom-Left (x, y) - slightly indented to remove bezel
        self.bl = (0, 480)   
        
        # Top-Left (x, y) - at the 60% mark
        self.tl = (0, 288)   
        
        # Top-Right (x, y) - at the 60% mark
        self.tr = (640, 288) 
        
        # Bottom-Right (x, y) - slightly indented to remove bezel
        self.br = (640, 480)
        
        # self.bl, self.tl, self.tr, self.br = (30, 477), (115, 247), (520, 255), (638, 473)
        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])    
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        # Tuning
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([179, 60, 255])
        self.canny_low = 50
        self.canny_high = 150
        self.color_weight = 0.7
        self.final_thresh = 50
        
        # Smoothing & Memory
        self.prevLx, self.prevRx = [], []
        self.alpha = 0.5
        
        # --- SANITY CHECK PARAMETERS ---
        self.EXPECTED_LANE_WIDTH = 450  # Pixels between L and R lines (Tune this!)
        self.LANE_WIDTH_TOLERANCE = 150 # Allow +/- 150 pixels variance
        
        self.last_valid_offset = 0.0
        self.last_valid_heading = 0.0
        self.consecutive_lost_frames = 0
        self.MAX_LOST_FRAMES = 10  # Hold last command for ~0.5 seconds (at 20Hz)

    def process_frame(self, frame):
        height, width = frame.shape[:2]
        
        # 1. PRE-PROCESSING
        bird_eye = cv2.warpPerspective(frame, self.matrix, (640, 480))
        gray = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2HSV)
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)

        # 2. FEATURE EXTRACTION
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        canny_edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        edges_dilated = cv2.dilate(canny_edges, np.ones((3,3), np.uint8), iterations=1)
        
        combined_prob = cv2.addWeighted(white_mask, self.color_weight, edges_dilated, 1.0 - self.color_weight, 0)
        _, mask = cv2.threshold(combined_prob, self.final_thresh, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        # 3. SLIDING WINDOW
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0]//2)
        
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
        ly_coords, ry_coords = [], [] 

        while y > 0:
            # Left
            win_l = mask[max(0, y-20):y, max(0, left_base-50):min(640, left_base+50)]
            cnts_l, _ = cv2.findContours(win_l, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts_l:
                c = max(cnts_l, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    left_base = left_base - 50 + int(M["m10"] / M["m00"])
                    lx.append(left_base)
                    ly_coords.append(y - 10)

            # Right
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

        # 4. VALIDATION & LOGIC
        valid_lane_found = False
        
        # Check A: Do we have enough points?
        if len(lx) > 3 and len(rx) > 3:
            
            # Check B: Is the width sane?
            avg_lx = np.mean(lx)
            avg_rx = np.mean(rx)
            calculated_width = avg_rx - avg_lx
            
            min_width = self.EXPECTED_LANE_WIDTH - self.LANE_WIDTH_TOLERANCE
            max_width = self.EXPECTED_LANE_WIDTH + self.LANE_WIDTH_TOLERANCE
            
            if min_width < calculated_width < max_width:
                valid_lane_found = True
                self.consecutive_lost_frames = 0 # Reset counter
                
                # Update Smoothing
                if self.prevLx and len(lx) == len(self.prevLx):
                    lx = (np.array(lx) * self.alpha + np.array(self.prevLx) * (1 - self.alpha)).astype(int).tolist()
                self.prevLx = lx
                if self.prevRx and len(rx) == len(self.prevRx):
                    rx = (np.array(rx) * self.alpha + np.array(self.prevRx) * (1 - self.alpha)).astype(int).tolist()
                self.prevRx = rx

                # Calc Offset
                lane_center_px = (lx[0] + rx[0]) / 2
                offset_norm = (lane_center_px - (width / 2)) / (width / 2)
                
                # Calc Heading
                slopes = []
                fit_l = np.polyfit(ly_coords[:len(lx)], lx, 1)
                fit_r = np.polyfit(ry_coords[:len(rx)], rx, 1)
                slopes.append(fit_l[0]) 
                slopes.append(fit_r[0])
                avg_slope_inv = np.mean(slopes)
                heading_rad = float(np.arctan(-avg_slope_inv))
                
                # Store valid values
                self.last_valid_offset = offset_norm
                self.last_valid_heading = heading_rad
            else:
                # Width check failed - Treat as noise
                pass 

        # 5. HANDLE LOST LANES (Memory Logic)
        if not valid_lane_found:
            self.consecutive_lost_frames += 1
            if self.consecutive_lost_frames < self.MAX_LOST_FRAMES:
                # Use memory
                offset_norm = self.last_valid_offset
                heading_rad = self.last_valid_heading
                
                # Visual Indicator for "Memory Mode"
                cv2.putText(frame, "⚠️ LANE LOST - USING MEMORY", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            else:
                # Lost for too long - Stop/Neutral
                offset_norm = 0.0
                heading_rad = 0.0
                cv2.putText(frame, "🛑 LANE LOST - STOP", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            offset_norm = self.last_valid_offset
            heading_rad = self.last_valid_heading

        # 6. VISUALIZATION
        overlay = bird_eye.copy()
        if valid_lane_found and len(lx) > 0 and len(rx) > 0:
            pts_left = np.array([[x, 480 - i*20] for i, x in enumerate(lx)])
            pts_right = np.array([[x, 480 - i*20] for i, x in enumerate(rx)])
            pts = np.vstack([pts_left, np.flipud(pts_right)])
            cv2.fillPoly(overlay, [np.int32(pts)], (0, 255, 0))

        unwarped_lane = cv2.warpPerspective(overlay, self.inv_matrix, (width, height))
        result = cv2.addWeighted(frame, 1, unwarped_lane, 0.5, 0)
        
        cv2.putText(result, f"Off: {offset_norm:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(result, f"Head: {heading_rad:.2f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return float(offset_norm), float(heading_rad), result


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
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (640, 480))

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