import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import os

class LaneDetectionNode(Node):
    """
    Real-time ROS2 lane detection (improved visualization version).
    Uses sliding window detection, publishes lane offset and heading,
    and optionally records an output video showing:
      - ROI (red lines)
      - Green filled lane polygon
      - Offset & heading text
    """

    def __init__(self):
        super().__init__('lane_detection_node')
        self.pub_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_heading = self.create_publisher(Float32, 'lane_heading', 10)

        # Parameters for recording
        self.declare_parameter('record', False)
        self.declare_parameter('output_path', '/home/rppi4/lane_output/lane_output.avi')
        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warning("⚠️ Camera not opened (index 0). Node still running.")
        self.latest_frame = None
        self.lock = threading.Lock()

        # Optional video writer
        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (640, 480))
            self.get_logger().info(f"🎥 Recording lane output to: {self.output_path}")
        else:
            self.get_logger().info("🧪 Recording disabled (record=false)")

        # Start camera capture in background thread
        threading.Thread(target=self._capture_frames, daemon=True).start()

        # Timer for lane detection loop (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    # ---------------- CAMERA THREAD ----------------
    def _capture_frames(self):
        """Continuously read frames from camera."""
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                with self.lock:
                    self.latest_frame = frame

    # ---------------- DETECTION ----------------
    def detect_lane_frame(self, frame):
        """Sliding window lane detection with proper polygon overlay and heading calculation."""
        height, width = frame.shape[:2]
        roi_points = np.array([[
            (0, height-50),
            (width, height-50),
            (width, int(height*0.5)),
            (0, int(height*0.5))
        ]], dtype=np.int32)

        # Draw ROI for visualization
        roi_frame = frame.copy()
        cv2.polylines(roi_frame, roi_points, isClosed=True, color=(0, 0, 255), thickness=1)

        # Perspective transform setup
        pts1 = np.float32([roi_points[0][3], roi_points[0][0], roi_points[0][2], roi_points[0][1]])  # tl, bl, tr, br
        pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
        warped = cv2.warpPerspective(frame, matrix, (width, height))

        # HSV threshold for blue lanes
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([86, 40, 0])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Sliding window detection
        lx, rx = self.sliding_window_lane(mask)
        if len(lx) == 0 or len(rx) == 0:
            return 0.0, 0.0, roi_frame

        # Fit polynomials for heading calculation
        left_fit = np.polyfit(np.arange(len(lx)), lx, 2) if len(lx) > 2 else None
        right_fit = np.polyfit(np.arange(len(rx)), rx, 2) if len(rx) > 2 else None

        # Compute heading (average of lane slopes)
        y_eval = height * 0.9
        heading_rad = 0.0
        if left_fit is not None and right_fit is not None:
            left_slope = 2 * left_fit[0] * y_eval + left_fit[1]
            right_slope = 2 * right_fit[0] * y_eval + right_fit[1]
            lane_slope = (left_slope + right_slope) / 2.0
            heading_rad = float(np.arctan(lane_slope))

        # Build lane polygon (for green filled overlay)
        min_length = min(len(lx), len(rx))
        top_left = (lx[0], height)
        bottom_left = (lx[min_length-1], 0)
        top_right = (rx[0], height)
        bottom_right = (rx[min_length-1], 0)
        quad_points = np.array([[top_left, bottom_left, bottom_right, top_right]], dtype=np.int32).reshape((-1,1,2))

        # Create lane mask and fill polygon
        lane_mask = np.zeros_like(warped)
        cv2.fillPoly(lane_mask, [quad_points], (0, 255, 0))

        # Warp mask back to original perspective
        unwarped_lane = cv2.warpPerspective(lane_mask, inv_matrix, (width, height))

        # Blend mask with original frame
        alpha = 0.3
        result = cv2.addWeighted(frame, 1, unwarped_lane, alpha, 0)

        # Draw ROI boundary for clarity
        cv2.polylines(result, roi_points, isClosed=True, color=(0, 0, 255), thickness=1)

        # Compute lateral offset
        lane_center = (np.mean(lx) + np.mean(rx)) / 2
        pixel_offset = lane_center - width / 2
        offset_norm = pixel_offset / (width / 2)

        # Display offset and heading
        cv2.putText(result, f"Offset: {offset_norm:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result, f"Heading: {heading_rad:.2f} rad", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return float(offset_norm), float(heading_rad), result

    # ---------------- SLIDING WINDOW ----------------
    def sliding_window_lane(self, mask):
        """Sliding window lane detection."""
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        n_windows = 12
        window_height = mask.shape[0] // n_windows
        nonzero = mask.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        margin, minpix = 50, 50
        lx, rx = [], []
        l_current, r_current = left_base, right_base

        for window in range(n_windows):
            win_y_low = mask.shape[0] - (window + 1) * window_height
            win_y_high = mask.shape[0] - window * window_height
            win_xleft_low, win_xleft_high = l_current - margin, l_current + margin
            win_xright_low, win_xright_high = r_current - margin, r_current + margin

            good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            if len(good_left) > minpix:
                l_current = int(np.mean(nonzerox[good_left]))
            if len(good_right) > minpix:
                r_current = int(np.mean(nonzerox[good_right]))

            lx.extend(nonzerox[good_left])
            rx.extend(nonzerox[good_right])

        return np.array(lx), np.array(rx)

    # ---------------- TIMER CALLBACK ----------------
    def timer_callback(self):
        with self.lock:
            frame = self.latest_frame
        if frame is None:
            return

        offset, heading, visual = self.detect_lane_frame(frame)

        # Publish data
        msg_offset = Float32()
        msg_offset.data = offset
        self.pub_offset.publish(msg_offset)

        msg_heading = Float32()
        msg_heading.data = heading
        self.pub_heading.publish(msg_heading)

        # Save to video if enabled
        if self.record and self.out and self.out.isOpened():
            self.out.write(visual)

        self.get_logger().info(f"Lane offset: {offset:.3f}, Heading: {heading:.3f}")

    # ---------------- CLEANUP ----------------
    def cleanup(self):
        self.get_logger().info("🧹 Releasing camera and video writer...")
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.out:
            self.out.release()

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
