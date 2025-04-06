import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Initialize camera
        gst_str = (
            "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! appsink"
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            return

        # ROS 2 publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # CV bridge for converting OpenCV -> ROS image
        self.bridge = CvBridge()

        # Timer callback
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Published image frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
