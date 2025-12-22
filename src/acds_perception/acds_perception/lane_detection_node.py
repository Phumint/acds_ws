#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

import cv2
import numpy as np
import threading
import os

from cv_bridge import CvBridge

# ==============================================================================
#  ALGORITHM CLASS (UNCHANGED)
# ==============================================================================

class LaneDetectionAlgorithm:
    def __init__(self):
        self.bl = (0, 480)
        self.tl = (0, 288)
        self.tr = (640, 288)
        self.br = (640, 480)

        self.pts1 = np.float32([self.tl, self.bl, self.tr, self.br])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.inv_matrix = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([179, 60, 255])
        self.canny_low = 50
        self.canny_high = 150
        self.color_weight = 0.7
        self.final_thresh = 50

        self.prevLx, self.prevRx = [], []
        self.alpha = 0.5

        self.EXPECTED_LANE_WIDTH = 450
        self.LANE_WIDTH_TOLERANCE = 150

        self.last_valid_offset = 0.0
        self.last_valid_heading = 0.0
        self.consecutive_lost_frames = 0
        self.MAX_LOST_FRAMES = 10

    def process_frame(self, frame):
        height, width = frame.shape[:2]

        bird_eye = cv2.warpPerspective(frame, self.matrix, (640, 480))
        gray = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2HSV)
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)

        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        canny_edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        edges_dilated = cv2.dilate(canny_edges, np.ones((3,3), np.uint8), 1)

        combined_prob = cv2.addWeighted(
            white_mask, self.color_weight,
            edges_dilated, 1.0 - self.color_weight, 0
        )

        _, mask = cv2.threshold(combined_prob, self.final_thresh, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2

        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > 0 else width // 4
        right_base = np.argmax(histogram[midpoint:]) + midpoint if np.max(histogram[midpoint:]) > 0 else 3 * width // 4

        y = mask.shape[0]
        lx, rx, ly, ry = [], [], [], []

        while y > 0:
            win_l = mask[max(0, y-20):y, max(0, left_base-50):min(640, left_base+50)]
            cnts_l, _ = cv2.findContours(win_l, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts_l:
                c = max(cnts_l, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"]:
                    left_base = left_base - 50 + int(M["m10"]/M["m00"])
                    lx.append(left_base)
                    ly.append(y-10)

            win_r = mask[max(0, y-20):y, max(0, right_base-50):min(640, right_base+50)]
            cnts_r, _ = cv2.findContours(win_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts_r:
                c = max(cnts_r, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"]:
                    right_base = right_base - 50 + int(M["m10"]/M["m00"])
                    rx.append(right_base)
                    ry.append(y-10)

            y -= 20

        valid_lane = False

        if len(lx) > 3 and len(rx) > 3:
            width_px = np.mean(rx) - np.mean(lx)
            if abs(width_px - self.EXPECTED_LANE_WIDTH) < self.LANE_WIDTH_TOLERANCE:
                valid_lane = True
                self.consecutive_lost_frames = 0

                fit_l = np.polyfit(ly, lx, 1)
                fit_r = np.polyfit(ry, rx, 1)

                lane_center = (lx[0] + rx[0]) / 2
                offset = (lane_center - width/2) / (width/2)
                heading = float(np.arctan(-np.mean([fit_l[0], fit_r[0]])))

                self.last_valid_offset = offset
                self.last_valid_heading = heading

        if not valid_lane:
            self.consecutive_lost_frames += 1
            if self.consecutive_lost_frames < self.MAX_LOST_FRAMES:
                offset = self.last_valid_offset
                heading = self.last_valid_heading
            else:
                offset = 0.0
                heading = 0.0
        else:
            offset = self.last_valid_offset
            heading = self.last_valid_heading

    # ===================== VISUALIZATION =====================
        overlay = bird_eye.copy()

        if valid_lane and len(lx) > 0 and len(rx) > 0:
            pts_left = np.array([[x, 480 - i*20] for i, x in enumerate(lx)])
            pts_right = np.array([[x, 480 - i*20] for i, x in enumerate(rx)])
            pts = np.vstack([pts_left, np.flipud(pts_right)])
            cv2.fillPoly(overlay, [np.int32(pts)], (0, 255, 0))

        unwarped = cv2.warpPerspective(overlay, self.inv_matrix, (width, height))
        result = cv2.addWeighted(frame, 1.0, unwarped, 0.5, 0)

        cv2.putText(result, f"Off: {offset:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(result, f"Head: {heading:.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


        return offset, heading, result


# ==============================================================================
#  ROS NODE (CS30 CAMERA)
# ==============================================================================

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.pub_offset = self.create_publisher(Float32, 'lane_offset', 10)
        self.pub_heading = self.create_publisher(Float32, 'lane_heading', 10)

        self.declare_parameter('camera_prefix', '/camera1_HV0130315L0317')
        self.declare_parameter('record', True)
        self.declare_parameter('output_path', '/home/rppi4/workspace/acds_ws/lane_output.avi')

        prefix = self.get_parameter('camera_prefix').value
        self.record = self.get_parameter('record').value
        self.output_path = self.get_parameter('output_path').value

        self.detector = LaneDetectionAlgorithm()
        self.bridge = CvBridge()

        self.latest_frame = None
        self.lock = threading.Lock()

        # CS30 RGB subscription
        self.create_subscription(
            Image,
            f'{prefix}/rgb_raw',
            self.rgb_callback,
            1
        )

        self.out = None
        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(self.output_path, fourcc, 20.0, (640, 480))

        self.create_timer(0.05, self.timer_callback)

    def rgb_callback(self, msg):
        try:
            if msg.encoding == '8SC3':
                img = np.frombuffer(msg.data, dtype=np.byte).reshape(
                    msg.height, msg.width, 3).astype(np.uint8)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            img = cv2.resize(img, (640, 480))

            with self.lock:
                self.latest_frame = img

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def timer_callback(self):
        with self.lock:
            frame = self.latest_frame

        if frame is None:
            return

        offset, heading, visual = self.detector.process_frame(frame)

        self.pub_offset.publish(Float32(data=float(offset)))
        self.pub_heading.publish(Float32(data=float(heading)))

        if self.record and self.out and self.out.isOpened():
            self.out.write(visual)

    def cleanup(self):
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
