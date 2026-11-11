import json
import math
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String


class YoloSiafDetector(Node):
    """Synchronize RGB/IR frames, run six-channel YOLO-SIAF, and estimate range."""

    def __init__(self):
        super().__init__("yolo_siaf_detector")
        self.declare_parameter("rgb_topic", "/sensors/rgb/image_raw")
        self.declare_parameter("ir_topic", "/sensors/ir/image_raw")
        self.declare_parameter("camera_info_topic", "/sensors/rgb/camera_info")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "0")
        self.declare_parameter("image_size", 640)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.65)
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("nominal_person_height_m", 1.70)
        self.declare_parameter("sync_slop_s", 0.08)
        self.declare_parameter("max_inference_rate_hz", 15.0)

        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            raise RuntimeError("model_path is empty; pass model_path:=/path/to/best.pt")

        # Import here so ROS diagnostics remain readable if the local repository is
        # missing from PYTHONPATH.
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                "Unable to import the modified Ultralytics package. Add the "
                "YOLOv11-RGBT-master directory to PYTHONPATH."
            ) from exc

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = None
        self.last_inference_monotonic = 0.0

        self.device = str(self.get_parameter("device").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.conf = float(self.get_parameter("confidence_threshold").value)
        self.iou = float(self.get_parameter("iou_threshold").value)
        self.person_class_id = int(self.get_parameter("person_class_id").value)
        self.person_height = float(self.get_parameter("nominal_person_height_m").value)
        max_rate = float(self.get_parameter("max_inference_rate_hz").value)
        self.minimum_period = 1.0 / max(max_rate, 0.1)

        self.distance_pub = self.create_publisher(
            Float32, "/yolo_siaf/nearest_pedestrian_distance", 10
        )
        self.detections_pub = self.create_publisher(String, "/yolo_siaf/detections", 10)
        self.annotated_pub = self.create_publisher(Image, "/yolo_siaf/annotated_image", 5)

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            10,
        )

        rgb_sub = message_filters.Subscriber(
            self, Image, str(self.get_parameter("rgb_topic").value)
        )
        ir_sub = message_filters.Subscriber(
            self, Image, str(self.get_parameter("ir_topic").value)
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, ir_sub],
            queue_size=8,
            slop=float(self.get_parameter("sync_slop_s").value),
        )
        sync.registerCallback(self._image_callback)
        # Keep references alive for the lifetime of the node.
        self.rgb_sub = rgb_sub
        self.ir_sub = ir_sub
        self.sync = sync
        self.get_logger().info(f"Loaded YOLO-SIAF checkpoint: {model_path}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 1 and msg.k[0] > 0.0:
            self.fx = float(msg.k[0])

    def _image_callback(self, rgb_msg: Image, ir_msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_inference_monotonic < self.minimum_period:
            return
        self.last_inference_monotonic = now

        try:
            rgb_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            ir_mono = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        if ir_mono.shape[:2] != rgb_bgr.shape[:2]:
            ir_mono = cv2.resize(
                ir_mono, (rgb_bgr.shape[1], rgb_bgr.shape[0]), interpolation=cv2.INTER_LINEAR
            )
        ir_bgr = cv2.cvtColor(ir_mono, cv2.COLOR_GRAY2BGR)
        six_channel = np.ascontiguousarray(np.concatenate((rgb_bgr, ir_bgr), axis=2))

        try:
            results = self.model.predict(
                source=six_channel,
                imgsz=self.image_size,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                channels=6,
                use_simotm="RGBRGB6C",
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO-SIAF inference failed: {exc}")
            return

        annotated = rgb_bgr.copy()
        records = []
        nearest = math.inf
        fx = self.fx if self.fx is not None else 554.3  # 640 px, 60 deg HFOV

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                class_id = int(box.cls.item())
                if class_id != self.person_class_id:
                    continue
                score = float(box.conf.item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                pixel_height = max(y2 - y1, 1.0)
                distance = fx * self.person_height / pixel_height
                nearest = min(nearest, distance)
                records.append(
                    {
                        "class_id": class_id,
                        "score": round(score, 4),
                        "bbox_xyxy": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                        "distance_m": round(distance, 2),
                    }
                )
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(annotated, p1, p2, (0, 220, 0), 2)
                cv2.putText(
                    annotated,
                    f"person {score:.2f}  {distance:.1f} m",
                    (p1[0], max(p1[1] - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 220, 0),
                    2,
                    cv2.LINE_AA,
                )

        if math.isfinite(nearest):
            self.distance_pub.publish(Float32(data=float(nearest)))
        self.detections_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": {
                            "sec": int(rgb_msg.header.stamp.sec),
                            "nanosec": int(rgb_msg.header.stamp.nanosec),
                        },
                        "detections": records,
                    }
                )
            )
        )
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = rgb_msg.header
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloSiafDetector()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
