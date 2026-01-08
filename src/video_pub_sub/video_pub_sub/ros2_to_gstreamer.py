# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge
# # import gi
# # gi.require_version('Gst', '1.0')
# # from gi.repository import Gst

# # class ROS2ToGStreamer(Node):
# #     def __init__(self):
# #         super().__init__('ros2_to_gstreamer')
# #         self.bridge = CvBridge()
# #         self.subscription = self.create_subscription(
# #             Image,
# #             '/camera/test_image',  # your ROS2 topic
# #             self.image_callback,
# #             10
# #         )
# #         Gst.init(None)

# #         # GStreamer pipeline: appsrc -> videoconvert -> x264enc -> mpegts -> udpsink
# #         pipeline_desc = (
# #             'appsrc name=source ! videoconvert ! '
# #             'x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 ! '
# #             'mpegtsmux ! '
# #             'udpsink host=127.0.0.1 port=5000'
# #         )
# #         self.pipeline = Gst.parse_launch(pipeline_desc)
# #         self.appsrc = self.pipeline.get_by_name('source')
# #         self.appsrc.set_property('format', Gst.Format.TIME)
# #         self.pipeline.set_state(Gst.State.PLAYING)

# #     def image_callback(self, msg):
# #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# #         gst_buffer = Gst.Buffer.new_wrapped(frame.tobytes())
# #         self.appsrc.emit('push-buffer', gst_buffer)
# #         self.get_logger().info('Published a frame to GStreamer')

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = ROS2ToGStreamer()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.pipeline.set_state(Gst.State.NULL)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# #!/usr/bin/env python3

# #!/usr/bin/env python3
# #!/usr/bin/env python3

# #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge
# # import gi
# # import cv2

# # gi.require_version('Gst', '1.0')
# # from gi.repository import Gst

# # class ROS2ToGStreamer(Node):
# #     def __init__(self):
# #         super().__init__('ros2_to_gstreamer')

# #         self.width = 854
# #         self.height = 480
# #         self.fps = 24
# #         self.bridge = CvBridge()
# #         self.timestamp = 0
# #         self.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, self.fps)

# #         self.subscription = self.create_subscription(
# #             Image,
# #             '/camera/test_image',
# #             self.image_callback,
# #             10
# #         )

# #         Gst.init(None)

# #         # ✅ Safe pipeline
# #         pipeline_desc = (
# #             f'appsrc name=source is-live=true block=true format=time '
# #             f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 '
# #             '! videoconvert '
# #             '! video/x-raw,format=I420 '
# #             '! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 key-int-max=24 '
# #             '! rtph264pay config-interval=1 pt=96 '
# #             '! udpsink host=127.0.0.1 port=5000'
# #         )

# #         self.pipeline = Gst.parse_launch(pipeline_desc)
# #         self.appsrc = self.pipeline.get_by_name('source')
# #         self.pipeline.set_state(Gst.State.PLAYING)
# #         self.get_logger().info("ROS2 → GStreamer pipeline started")

# #     def image_callback(self, msg):
# #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

# #         if frame.shape[1] != self.width or frame.shape[0] != self.height:
# #             frame = cv2.resize(frame, (self.width, self.height))

# #         # Convert frame to bytes
# #         data = frame.tobytes()
# #         buf = Gst.Buffer.new_allocate(None, len(data), None)
# #         buf.fill(0, data)

# #         buf.pts = buf.dts = self.timestamp
# #         buf.duration = self.duration
# #         self.timestamp += self.duration

# #         retval = self.appsrc.emit('push-buffer', buf)
# #         if retval != Gst.FlowReturn.OK:
# #             self.get_logger().error('GStreamer push-buffer failed')


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = ROS2ToGStreamer()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.pipeline.set_state(Gst.State.NULL)
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# #!/usr/bin/env python3
# #!/usr/bin/env python3
# #!/usr/bin/env python3
# #!/usr/bin/env python3

# #!/usr/bin/env python3

# import sys
# import os

# # Force system GStreamer paths FIRST
# os.environ['GI_TYPELIB_PATH'] = '/usr/lib/x86_64-linux-gnu/gstreamer-1.0'
# os.environ['GST_PLUGIN_SYSTEM_PATH'] = '/usr/lib/x86_64-linux-gnu/gstreamer-1.0'

# # GStreamer imports FIRST - prevents segfault
# import gi
# gi.require_version('Gst', '1.0')
# gi.require_version('GstWebRTC', '1.0')
# from gi.repository import Gst, GstWebRTC, GLib
# Gst.init(None)

# # ROS2 imports AFTER
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import numpy as np

# class Ros2WebRTCPublisher(Node):
#     def __init__(self):
#         super().__init__('ros2_to_webrtc')
#         self.bridge = CvBridge()
        
#         self.width, self.height, self.fps = 640, 480, 30
#         self.timestamp = 0

#         # FIXED PIPELINE - removed invalid config-interval, proper WebRTC setup
#         self.pipeline_str = (
#             f'appsrc name=mysrc is-live=true format=time do-timestamp=true '
#             f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! '
#             'videoconvert ! '
#             'alpha method=green ! '
#             'videoconvert ! '
#             'vp8enc deadline=1 cpu-used=5 ! '
#             'rtpvp8pay pt=97 ! '  # FIXED: no config-interval property
#             'webrtcbin name=sendrecv stun-server=stun://stun.l.google.com:19302'
#         )

#         print(f"Creating pipeline: {self.pipeline_str}")  # Debug
        
#         self.pipeline = Gst.parse_launch(self.pipeline_str)
        
#         if not self.pipeline:
#             self.get_logger().error("Failed to create pipeline")
#             self.pipeline = None
#             return

#         self.appsrc = self.pipeline.get_by_name('mysrc')
#         self.webrtc = self.pipeline.get_by_name('sendrecv')

#         ret = self.pipeline.set_state(Gst.State.PLAYING)
#         if ret == Gst.StateChangeReturn.FAILURE:
#             self.get_logger().error("Pipeline failed to PLAY")
#             self.pipeline = None
#             return
            
#         self.get_logger().info("✅ WebRTC pipeline PLAYING")

#         self.sub = self.create_subscription(
#             Image, '/camera/test_image', self.ros_callback, 10
#         )

#     def ros_callback(self, msg):
#         if not self.appsrc or not self.pipeline:
#             return
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             data = frame.tobytes()
#             buf = Gst.Buffer.new_wrapped(data)
            
#             duration = int(1e9 / self.fps)
#             buf.pts = self.timestamp
#             buf.duration = duration
#             self.timestamp += duration

#             self.appsrc.emit('push-buffer', buf)
#         except Exception as e:
#             self.get_logger().error(f"Callback error: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = Ros2WebRTCPublisher()
#     try:
#         if hasattr(node, 'pipeline') and node.pipeline:
#             rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if hasattr(node, 'pipeline') and node.pipeline:
#             node.pipeline.set_state(Gst.State.NULL)
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3
#!/usr/bin/env python3

#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import queue
import threading
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer globally
Gst.init(None)

class ROS2ToGStreamer(Node):
    def __init__(self):
        super().__init__('ros2_to_gstreamer')

        # ---------------- CONFIG ----------------
        self.width = 640
        self.height = 480
        self.fps = 30
        self.bitrate = 4000  # kbps
        self.udp_host = "127.0.0.1"  # Docker container name of MediaMTX
        self.udp_port = 8002
        # ----------------------------------------

        self.bridge = CvBridge()
        self.timestamp = 0
        self.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, self.fps)

        # Thread-safe queue for frames
        self.frame_queue = queue.Queue(maxsize=10)

        # ---------------- ROS SUBSCRIPTION ----------------
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_detected',  # ROS2 topic
            self.ros_callback,
            10
        )

        # ---------------- GSTREAMER PIPELINE ----------------
        # Use a single f-string for the entire multiline description
        pipeline_desc = (
            f"appsrc name=source is-live=true block=true format=time do-timestamp=true "
            f"! video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 "
            f"! videorate "
            f"! videoconvert " 
            f"! queue max-size-buffers=2 leaky=downstream "
            f"! video/x-raw,format=I420 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast "
            f"bitrate={self.bitrate} key-int-max={self.fps} bframes=0 "
            f"! h264parse config-interval=1 "
            f"! mpegtsmux "
            f"! identity sync=true "
            f"! udpsink host=mediamtx port=8002 sync=false async=false"
        )




        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name("source")
        if self.appsrc is None:
            raise RuntimeError("❌ appsrc element not found in pipeline")

        # Start GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        self.get_logger().info("✅ GStreamer pipeline started")

        # ---------------- GLib MAIN LOOP ----------------
        self.mainloop = GLib.MainLoop()
        threading.Thread(target=self.mainloop.run, daemon=True).start()

        # Push frames from queue every millisecond
        GLib.timeout_add(1, self.push_frame_from_queue)

    # ---------------- ROS CALLBACK ----------------
    def ros_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize if needed
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))

            if not self.frame_queue.full():
                self.frame_queue.put(frame)

        except Exception as e:
            self.get_logger().error(f"ROS callback error: {e}")

    # ---------------- PUSH TO GSTREAMER ----------------
    def push_frame_from_queue(self):
        if not self.frame_queue.empty():
            frame = self.frame_queue.get()
            buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
            buf.fill(0, frame.tobytes())
            buf.pts = buf.dts = self.timestamp
            buf.duration = self.duration
            self.timestamp += self.duration

            ret = self.appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                self.get_logger().warn(f"GStreamer push-buffer returned {ret}")

        return True  # continue calling

def main():
    rclpy.init()
    node = ROS2ToGStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.mainloop.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
