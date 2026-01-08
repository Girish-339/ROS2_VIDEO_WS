# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO

# class YoloDetector(Node):
#     def __init__(self):
#         super().__init__('yolo_detector')

#         self.sub = self.create_subscription(
#             Image,
#             '/camera/image',
#             self.callback,
#             10
#         )

#         self.pub = self.create_publisher(
#             Image,
#             '/camera/image_detected',
#             10
#         )

#         self.bridge = CvBridge()
#         self.model = YOLO('yolov8n.pt')  # CPU model

#         self.get_logger().info('✅ YOLO Detector started')

#     def callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#         results = self.model(frame, conf=0.4)
#         annotated = results[0].plot()

#         out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
#         out_msg.header = msg.header

#         self.pub.publish(out_msg)

# def main():
#     rclpy.init()
#     node = YoloDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO

# class YoloDetector(Node):
#     def __init__(self):
#         super().__init__('yolo_detector')

#         self.sub = self.create_subscription(
#             Image,
#             '/camera/image',
#             self.callback,
#             10
#         )

#         self.pub = self.create_publisher(
#             Image,
#             '/camera/image_detected',
#             10
#         )

#         self.bridge = CvBridge()
#         self.model = YOLO('yolov8n.pt')  # pretrained COCO model
#         # Define target classes by name (COCO)
#         self.target_classes = ['bicycle', 'tree']  # replace 'tree' if not in COCO

#         self.get_logger().info('✅ YOLO Detector started')

#     def callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#         # Run detection
#         results = self.model(frame, conf=0.2)

#         # Filter detections by class names
#         annotated = frame.copy()
#         for det in results[0].boxes:
#             cls_id = int(det.cls[0])
#             cls_name = self.model.names[cls_id]

#             if cls_name in self.target_classes:
#                 # Draw box + label
#                 x1, y1, x2, y2 = map(int, det.xyxy[0])
#                 cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(annotated, cls_name, (x1, y1-10),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

#         out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
#         out_msg.header = msg.header
#         self.pub.publish(out_msg)

# def main():
#     rclpy.init()
#     node = YoloDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Subscribe to camera images
        self.sub = self.create_subscription(
            Image,
            '/camera/image',
            self.callback,
            10
        )

        # Publisher for annotated images
        self.pub = self.create_publisher(
            Image,
            '/camera/image_detected',
            10
        )

        self.bridge = CvBridge()

        # Load YOLO nano model (lightweight, fast)
        self.model = YOLO('yolov8n.pt')  # <--- Nano model
        self.conf_threshold = 0.3

        self.get_logger().info('✅ YOLO Nano Detector started and ready')

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Run inference (verbose=False keeps Docker logs clean)
            results = self.model(frame, conf=self.conf_threshold, verbose=False)

            # Copy frame for drawing
            annotated = frame.copy()

            # Extract boxes from results
            boxes = results[0].boxes

            if boxes is not None:
                for box in boxes:
                    # Convert xyxy tensor to numpy ints
                    coords = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = coords

                    # Get class ID and confidence
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    # Get class name from model
                    class_name = self.model.names[cls_id]
                    label = f"{class_name} {conf:.2f}"

                    # Draw green bounding box
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label background
                    label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    y1_label = max(y1, label_size[1])
                    cv2.rectangle(annotated, (x1, y1_label - label_size[1] - 5),
                                  (x1 + label_size[0], y1_label + base_line), (0, 255, 0), cv2.FILLED)

                    # Put label text
                    cv2.putText(
                        annotated,
                        label,
                        (x1, y1_label - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2
                    )

            # Publish annotated image
            out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error in YOLO callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
