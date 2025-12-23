import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading
import os
from cv_bridge import CvBridge
from ultralytics import YOLO


class TrafficSignNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_node')

        # 1. Parameters
        self.declare_parameter('camera_prefix', '/camera1_HV0130315L0317')
        self.declare_parameter(
            'model_path',
            '/home/rppi4/workspace/acds_ws/src/acds_perception/acds_perception/models/best.pt'
        )
        self.declare_parameter('record', True)
        self.declare_parameter(
            'output_path',
            '/home/rppi4/workspace/acds_ws/sign_output.avi'
        )

        prefix = self.get_parameter('camera_prefix').value
        model_path = self.get_parameter('model_path').value
        self.record = self.get_parameter('record').value
        self.output_path = self.get_parameter('output_path').value

        # 2. Initialization
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.latest_frame = None
        self.lock = threading.Lock()

        self.out = None
        self.frame_count = 0

        if self.record:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)

        # 3. ROS Communication
        self.pub_id = self.create_publisher(Int32, 'traffic_sign_id', 10)
        self.pub_label = self.create_publisher(String, 'traffic_sign_label', 10)

        self.create_subscription(
            Image,
            f'{prefix}/rgb_raw',
            self.rgb_callback,
            10
        )

        # YOLO inference at 10 FPS
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("TrafficSignNode started")

    # ----------------------------------------------------------
    # Camera callback (FAST, no heavy work)
    # ----------------------------------------------------------
    def rgb_callback(self, msg):
        try:
            if msg.encoding == '8SC3':
                img = np.frombuffer(
                    msg.data, dtype=np.byte
                ).reshape(msg.height, msg.width, 3).astype(np.uint8)
            else:
                img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8'
                )

            img = cv2.resize(img, (640, 480))

            with self.lock:
                self.latest_frame = img.copy()

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    # ----------------------------------------------------------
    # Timer callback (YOLO + overlay + recording)
    # ----------------------------------------------------------
    def timer_callback(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            return

        # Run YOLO inference
        results = self.model(frame, conf=0.25, imgsz=320, verbose=False)

        detections = []

        if len(results[0].boxes) > 0:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                conf = float(box.conf[0])

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                detections.append(
                    (cls_id, label, conf, (x1, y1, x2, y2))
                )

            # Publish the most confident detection only
            best = max(detections, key=lambda d: d[2])
            self.pub_id.publish(Int32(data=best[0]))
            self.pub_label.publish(String(data=best[1]))

        # Draw detections
        for cls_id, label, conf, (x1, y1, x2, y2) in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{label} {conf:.2f}",
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # Initialize VideoWriter lazily (guarantees correct size)
        if self.record:
            if self.out is None:
                h, w, _ = frame.shape
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                self.out = cv2.VideoWriter(
                    self.output_path,
                    fourcc,
                    10.0,
                    (w, h)
                )
                self.get_logger().info(
                    f"VideoWriter started ({w}x{h})"
                )

            self.out.write(frame)
            self.frame_count += 1

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Recorded {self.frame_count} frames"
                )

    # ----------------------------------------------------------
    # Cleanup
    # ----------------------------------------------------------
    def cleanup(self):
        if self.out is not None:
            self.out.release()
            self.out = None
            self.get_logger().info(
                f"Video file saved. Total frames: {self.frame_count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
