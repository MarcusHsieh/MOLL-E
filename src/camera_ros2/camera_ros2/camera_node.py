import cv2
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        gst_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink"
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)  

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera or video")
        else:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"FPS: {fps}")
        
        self.fps_publisher = self.create_publisher(Float32, 'camera_fps', 10)
        self.timer = self.create_timer(1.0, self.publish_fps)
    
    def publish_fps(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        msg = Float32()
        msg.data = fps
        self.fps_publisher.publish(msg)
        self.get_logger().info(f"FPS: {fps}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Camera controller node...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()