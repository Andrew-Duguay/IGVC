#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles  # <-- add this

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2


class YoloTrtNode(Node):
    def __init__(self):
        super().__init__("yolo_trt_node")

        # Parameters (can override via ROS params)
        self.declare_parameter("weights", "yolo11n.engine")
        self.declare_parameter("img_size", 640)
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("device", 0)  # GPU index

        weights = self.get_parameter("weights").get_parameter_value().string_value
        self.img_size = self.get_parameter("img_size").get_parameter_value().integer_value
        self.conf = float(self.get_parameter("conf").get_parameter_value().double_value)
        self.device = int(self.get_parameter("device").get_parameter_value().integer_value)

        self.get_logger().info(f"Loading YOLO TensorRT engine: {weights}")
        self.model = YOLO(weights)  # This will load your .engine file

        self.bridge = CvBridge()

        qos = QoSPresetProfiles.SENSOR_DATA.value

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            "/image_raw",   # topic from v4l2_camera
            self.image_callback,
            10,
        )

        # Publish annotated images for visualization
        self.image_pub = self.create_publisher(
            Image,
            "/yolo/image_annotated",
            10,
        )

        # Publish structured detections for downstream nodes
        self.det_pub = self.create_publisher(
            Detection2DArray,
            "/yolo/detections",
            10,
        )

        self.get_logger().info(
            f"YOLO TRT node ready. Subscribing to /camera/image_raw, "
            f"publishing /yolo/image_annotated"
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(
            frame,
            imgsz=self.img_size,
            conf=self.conf,
            device=self.device,
        )
        result = results[0]

        # Build and publish Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header
        if result.boxes is not None:
            for box in result.boxes:
                det = Detection2D()
                det.header = msg.header
                # xyxy -> center + size
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                hyp.hypothesis.score = float(box.conf[0].item())
                det.results.append(hyp)
                det_array.detections.append(det)
        self.det_pub.publish(det_array)

        annotated = result.plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

        n = len(result.boxes) if result.boxes is not None else 0
        self.get_logger().info(f"Detections this frame: {n}")
def main(args=None):
    rclpy.init(args=args)
    node = YoloTrtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
