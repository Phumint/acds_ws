import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import os

class LaneDetectionNode(Node):
    """
    Lane detection with sliding window and T-section detection.
    Publishes lane offset & heading angle, draws visualization,
    and records output video.
    """

    def __init__(self):
        super().__init__('lane_detection_node')
        self.pub_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_heading = self.create_publisher(Float32, 'lane_heading', 10)

        # --- Parameters ---
        self.declare_parameter('record', False)
        self.declare_parameter('output_path', '/home/rppi4/lane_output/lane_output.avi')
        self.declare_parameter('t_section_turn', 'right')  # left or right

        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.t_section_turn = self.get_parameter('t_section_turn').get_parameter_value().string_value.lower()

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warning("⚠️ Camera not opened (index 0). Node still running.")
        self.latest_frame = None
        self.lock = threading.Lock()

        # --- Video writer setup ---
        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (640, 480))
            self.get_logger().info(f"🎥 Recording lane output to: {self.output_path}")
        else:
            self.get_logger().info("🧪 Recording disabled (record=false)")

        # --- Background thread for camera ---
        threading.Thread(target=self._capture_frames, daemon=True).start()

        # --- Timer for processing loop (20 Hz) ---
        self.timer = self.create_timer(0.05, self.timer_callback)

    # --------------------------------------------------
    def _capture_frames(self):
        """Continuously read frames from camera."""
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                with self.lock:
                    self.latest_frame = frame

    # --------------------------------------------------
    def detect_lane_frame(self, frame):
        """Sliding window lane detection + T-section handling."""
        height, width = frame.shape[:2]

        # --- Region of Interest (modifiable) ---
        roi_points = np.array([[
            (0, height - 50),
            (width, height - 50),
            (width, int(height * 0.5)),
            (0, int(height * 0.5))
        ]], dtype=np.int32)

        roi_frame = frame.copy()
        cv2.polylines(roi_frame, roi_points, isClosed=True, color=(0, 0, 255), thickness=1)

        # --- Perspective Transform ---
        pts1 = np.float32([roi_points[0][3], roi_points[0][0], roi_points[0][2], roi_points[0][1]])  # tl, bl, tr, br
        pts2 = np.float32([[0, 0], [0, height], [width, 0], [width, height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
        warped = cv2.warpPerspective(frame, matrix, (width, height))

        # --- Mask for blue lanes ---
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([86, 40, 0])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # --- Sliding window search ---
        lx, rx = self.sliding_window_lane(mask)
        t_section_detected = (len(lx) < 100 and len(rx) < 100)

        if t_section_detected:
            # T-section: perform configured turn
            result = frame.copy()
            offset_norm = 0.0
            if self.t_section_turn == 'right':
                heading_rad = -np.pi / 4   # Turn 45° right
            elif self.t_section_turn == 'left':
                heading_rad = np.pi / 4    # Turn 45° left
            else:
                heading_rad = 0.0  # default safe

            cv2.putText(result, f"T-section detected → Turning {self.t_section_turn.upper()}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.polylines(result, roi_points, isClosed=True, color=(0, 0, 255), thickness=1)

            self.get_logger().info(f"T-section detected → Turning {self.t_section_turn}")
            return float(offset_norm), float(heading_rad), result

        # --- Compute heading from lane curvature ---
        left_fit = np.polyfit(np.arange(len(lx)), lx, 2) if len(lx) > 2 else None
        right_fit = np.polyfit(np.arange(len(rx)), rx, 2) if len(rx) > 2 else None

        y_eval = height * 0.9
        heading_rad = 0.0
        if left_fit is not None and right_fit is not None:
            left_slope = 2 * left_fit[0] * y_eval + left_fit[1]
            right_slope = 2 * right_fit[0] * y_eval + right_fit[1]
            lane_slope = (left_slope + right_slope) / 2.0
            heading_rad = float(np.arctan(lane_slope))

        # --- Lane polygon ---
        min_length = min(len(lx), len(rx))
        top_left = (lx[0], height)
        bottom_left = (lx[min_length - 1], 0)
        top_right = (rx[0], height)
        bottom_right = (rx[min_length - 1], 0)
        quad_points = np.array([[top_left, bottom_left, bottom_right, top_right]], dtype=np.int32).reshape((-1, 1, 2))

        lane_mask = np.zeros_like(warped)
        cv2.fillPoly(lane_mask, [quad_points], (0, 255, 0))
        unwarped_lane = cv2.warpPerspective(lane_mask, inv_matrix, (width, height))

        alpha = 0.3
        result = cv2.addWeighted(frame, 1, unwarped_lane, alpha, 0)
        cv2.polylines(result, roi_points, isClosed=True, color=(0, 0, 255), thickness=1)

        lane_center = (np.mean(lx) + np.mean(rx)) / 2
        pixel_offset = lane_center - width / 2
        offset_norm = pixel_offset / (width / 2)

        cv2.putText(result, f"Offset: {offset_norm:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result, f"Heading: {heading_rad:.2f} rad", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return float(offset_norm), float(heading_rad), result

    # --------------------------------------------------
    def sliding_window_lane(self, mask):
        """Sliding window lane detection algorithm."""
        histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
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

    # --------------------------------------------------
    def timer_callback(self):
        """Main timer loop: process frame, publish offset & heading."""
        with self.lock:
            frame = self.latest_frame
        if frame is None:
            return

        offset, heading, visual = self.detect_lane_frame(frame)

        # Publish results
        msg_offset = Float32()
        msg_offset.data = offset
        self.pub_offset.publish(msg_offset)

        msg_heading = Float32()
        msg_heading.data = heading
        self.pub_heading.publish(msg_heading)

        # Record if enabled
        if self.record and self.out and self.out.isOpened():
            self.out.write(visual)

        self.get_logger().info(f"Lane offset: {offset:.3f}, Heading: {heading:.3f}")

    # --------------------------------------------------
    def cleanup(self):
        """Release hardware safely."""
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