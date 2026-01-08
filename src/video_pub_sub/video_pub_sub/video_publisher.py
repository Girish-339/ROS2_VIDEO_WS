# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge

# class VideoPublisher(Node):
#     def __init__(self):
#         super().__init__('video_publisher')
#         self.publisher_ = self.create_publisher(Image, '/camera/test_image', 10)
#         self.bridge = CvBridge()
#         self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
#         self.cap = cv2.VideoCapture('video.mp4')  # Path to your video

#     def timer_callback(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#             ret, frame = self.cap.read()
#         msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#         self.publisher_.publish(msg)
#         self.get_logger().info('Published a frame')

# def main(args=None):
#     rclpy.init(args=args)
#     node = VideoPublisher()
#     rclpy.spin(node)
#     node.cap.release()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time

class VideoFilePublisher(Node):
    def __init__(self):
        super().__init__('video_file_publisher')

        self.publisher = self.create_publisher(Image, '/video/image', 10)
        self.bridge = CvBridge()

         # 🔴 CHANGE PATH
        self.cap = cv2.VideoCapture('video.mp4')

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open video file")

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.fps <= 0:
            self.fps = 30

        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info("✅ video.mp4 → ROS Image started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("🔁 Looping video")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
