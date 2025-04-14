#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import sys
import signal

class GstCameraNode(Node):
    def __init__(self):
        super().__init__('gst_camera_node')

        # --- Parameters ---
        self.declare_parameter('camera_device', '/dev/video1')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('topic_name', '/camera/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera_link')
        # Allow overriding the whole pipeline
        self.declare_parameter('gst_pipeline_string', '')

        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.pipeline_str_override = self.get_parameter('gst_pipeline_string').get_parameter_value().string_value

        self.get_logger().info(f"Using camera device: {self.camera_device}")
        self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        self.get_logger().info(f"Publishing topic: {self.topic_name}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")

        # --- QoS Profile ---
        # Best effort for camera streams to avoid buffering delays on loss
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 # Keep only the latest frame
        )

        # --- Publisher ---
        self.publisher_ = self.create_publisher(CompressedImage, self.topic_name, qos_profile)

        # --- GStreamer Initialization ---
        Gst.init(sys.argv)
        self.pipeline = None
        self.appsink = None
        self.mainloop = GLib.MainLoop()

        # Handle Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # --- Build Pipeline ---
        self.build_pipeline()

    def build_pipeline(self):
        if self.pipeline_str_override:
             pipeline_str = self.pipeline_str_override
             self.get_logger().info(f"Using overridden GStreamer pipeline: {pipeline_str}")
        else:
            # Default pipeline based on parameters
            pipeline_str = (
                f"v4l2src device={self.camera_device} ! "
                f"image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! "
                f"appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false"
            )
            self.get_logger().info(f"Using generated GStreamer pipeline: {pipeline_str}")

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            self.get_logger().error(f"Failed to parse pipeline: {e}")
            rclpy.shutdown()
            sys.exit(1)

        # Get the appsink element
        self.appsink = self.pipeline.get_by_name('ros_sink')
        if not self.appsink:
            self.get_logger().error("Could not find appsink element 'ros_sink' in the pipeline.")
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            rclpy.shutdown()
            sys.exit(1)

        # Set the callback for new samples
        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self.on_new_sample)

    def start_pipeline(self):
        if self.pipeline:
            self.get_logger().info("Setting pipeline to PLAYING...")
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error("Unable to set the pipeline to the playing state.")
                self.cleanup()
                rclpy.shutdown()
                sys.exit(1)
            elif ret == Gst.StateChangeReturn.ASYNC:
                 self.get_logger().info("Pipeline state change is asynchronous.")
                 # We might need to wait for state change completion in some cases,
                 # but for simple pipelines, often starting the main loop is enough.
            elif ret == Gst.StateChangeReturn.NO_PREROLL:
                 self.get_logger().warning("Pipeline is live and does not need preroll.")
            else:
                 self.get_logger().info("Pipeline state set to PLAYING.")

            # Start the GLib main loop in a separate thread
            import threading
            self.mainloop_thread = threading.Thread(target=self.mainloop.run)
            self.mainloop_thread.start()
            self.get_logger().info("GStreamer main loop started.")
        else:
            self.get_logger().error("Pipeline not initialized.")

    def on_new_sample(self, appsink):
        """Callback function called when a new sample is available."""
        sample = appsink.pull_sample()
        if sample:
            try:
                buffer = sample.get_buffer()
                map_info = buffer.map(Gst.MapFlags.READ)

                # Create CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg" # Directly from the pipeline element
                msg.data = map_info.data # Get data as bytes

                # Publish the message
                self.publisher_.publish(msg)

                # Release the buffer map
                buffer.unmap(map_info)

            except Exception as e:
                self.get_logger().error(f"Error processing GStreamer sample: {e}", throttle_duration_sec=5)
            finally:
                # The sample object itself doesn't need explicit cleanup here
                # The buffer reference is managed within the sample.
                 pass # Keep this structure for potential future needs
        return Gst.FlowReturn.OK # Indicate success to GStreamer

    def cleanup(self):
        self.get_logger().info("Cleaning up GStreamer pipeline...")
        if hasattr(self, 'mainloop') and self.mainloop.is_running():
            self.mainloop.quit()
        if hasattr(self, 'mainloop_thread') and self.mainloop_thread.is_alive():
             self.mainloop_thread.join(timeout=2) # Wait max 2 seconds
             if self.mainloop_thread.is_alive():
                 self.get_logger().warning("Mainloop thread did not exit cleanly.")

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsink = None
        self.get_logger().info("GStreamer cleanup complete.")

    def signal_handler(self, sig, frame):
        self.get_logger().info('Ctrl+C detected. Shutting down.')
        self.cleanup()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = GstCameraNode()
    try:
        node.start_pipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt caught, shutting down.')
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in main: {e}")
    finally:
        # Ensure cleanup happens even if spin exits unexpectedly
        node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()