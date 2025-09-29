import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading

class LaneDetectionNode(Node):
    """
    Real-time ROS2 lane detection with threaded camera capture and sliding window.
    Publishes lane offset (normalized) to 'lane_offset'.
    """
    def __init__(self):
        super().__init__('lane_detection_node')
        self.publisher_ = self.create_publisher(Float32, 'lane_offset', 10)

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warning("Camera not opened (index 0). Node still running.")
        self.latest_frame = None
        self.lock = threading.Lock()

        # Start camera capture thread
        threading.Thread(target=self._capture_frames, daemon=True).start()

        # Timer for lane detection loop (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _capture_frames(self):
        """Continuously read frames from the camera in a separate thread."""
        while True:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame

    def detect_lane_frame(self, frame):
        """Sliding window lane detection algorithm."""
        height, width = frame.shape[:2]

        # Region of interest
        roi_points = np.array([[
            (0, height-50),
            (width, height-50),
            (width, int(height*0.5)),
            (0, int(height*0.5))
        ]], dtype=np.int32)

        # Perspective transform
        pts1 = np.float32([roi_points[0][3], roi_points[0][0],
                           roi_points[0][2], roi_points[0][1]])  # tl, bl, tr, br
        pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        warped = cv2.warpPerspective(frame, matrix, (width, height))

        # HSV threshold for blue lanes
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([86, 40, 0])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Sliding window detection
        lx, rx = self.sliding_window_lane(mask)
        if len(lx) == 0 or len(rx) == 0:
            return 0.0, 0.0, 0.0, frame

        # Polynomial fit
        left_fit = np.polyfit(np.arange(len(lx)), lx, 2) if len(lx) > 2 else None
        right_fit = np.polyfit(np.arange(len(rx)), rx, 2) if len(rx) > 2 else None

        # Lane heading calculation
        y_eval = height * 0.9
        heading_rad = 0.0
        if left_fit is not None and right_fit is not None:
            left_slope = 2*left_fit[0]*y_eval + left_fit[1]
            right_slope = 2*right_fit[0]*y_eval + right_fit[1]
            lane_slope = (left_slope + right_slope) / 2.0
            heading_rad = float(np.arctan(lane_slope))

        # Lateral offset
        lane_center = (np.mean(lx) + np.mean(rx)) / 2
        pixel_offset = lane_center - (width / 2)
        offset_norm = pixel_offset / (width / 2)

        return float(offset_norm), float(heading_rad), 1.0, frame

    def sliding_window_lane(self, mask):
        """Sliding window lane detection"""
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

    def timer_callback(self):
        """Process latest frame and publish lane offset"""
        with self.lock:
            frame = self.latest_frame
        if frame is None:
            return

        offset, heading, conf, _ = self.detect_lane_frame(frame)

        msg = Float32()
        msg.data = offset
        self.publisher_.publish(msg)
        self.get_logger().info(f"Lane offset: {offset:.3f}, Heading: {heading:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.cap and node.cap.isOpened():
                node.cap.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
