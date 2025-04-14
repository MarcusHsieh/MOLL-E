#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import gi
import threading
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class CsiCamPublisher(Node):

    def __init__(self):
        super().__init__('csi_cam_publisher')

        # Declare parameters
        self.declare_parameter('topic_name', '/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('display_width', 1280) # Often same as capture, adjust if scaling
        self.declare_parameter('display_height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('flip_method', 0) # 0=none, 1=counter-clockwise, 2=rotate-180, 3=clockwise, 4=horizontal-flip, 5=upper-right-diagonal, 6=vertical-flip, 7=upper-left-diagonal
        self.declare_parameter('format', 'jpeg') # 'jpeg' or 'png'
        self.declare_parameter('jpeg_quality', 90) # 0-100 for JPEG

        # Get parameters
        self.topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.capture_width = self.get_parameter('capture_width').value
        self.capture_height = self.get_parameter('capture_height').value
        self.display_width = self.get_parameter('display_width').value
        self.display_height = self.get_parameter('display_height').value
        self.framerate = self.get_parameter('framerate').value
        self.flip_method = self.get_parameter('flip_method').value
        self.format = self.get_parameter('format').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        if self.format not in ['jpeg', 'png']:
            self.get_logger().error(f"Unsupported format '{self.format}'. Defaulting to 'jpeg'.")
            self.format = 'jpeg'

        self.imencode_param = None
        if self.format == 'jpeg':
             self.imencode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        elif self.format == 'png':
             # PNG quality is 0-9, 0=no compression, 9=max compression. Default is often 3.
             self.imencode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]

        # Create publisher
        self.publisher_ = self.create_publisher(CompressedImage, self.topic_name, 10)
        self.get_logger().info(f"Publishing compressed images to topic: {self.topic_name}")

        # Initialize GStreamer
        Gst.init(None)
        self.pipeline = None
        self.appsink = None
        self.mainloop = GLib.MainLoop() # GLib's Main Loop
        self.mainloop_thread = threading.Thread(target=self._mainloop_run) # Run GLib loop in separate thread

        # Start GStreamer pipeline construction and main loop thread
        self.start_gst_pipeline()
        self.mainloop_thread.start()

    def _mainloop_run(self):
        self.get_logger().info("Starting GLib main loop for GStreamer...")
        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            self.get_logger().info("GLib main loop keyboard interrupt.")
        finally:
            self.get_logger().info("GLib main loop stopped.")

    def gstreamer_pipeline_string(self):
        return (
            f"nvarguscamerasrc sensor-mode=-1 ! " # sensor-mode=-1 lets driver pick best mode
            f"video/x-raw(memory:NVMM), width=(int){self.capture_width}, height=(int){self.capture_height}, framerate=(fraction){self.framerate}/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, width=(int){self.display_width}, height=(int){self.display_height}, format=(string)BGRx ! "
            f"videoconvert ! " # Convert from BGRx to BGR
            f"video/x-raw, format=(string)BGR ! "
            f"appsink name=appsink emit-signals=true max-buffers=1 drop=true" # Emit signal, limit buffer, drop old frames
        )

    def start_gst_pipeline(self):
        pipeline_str = self.gstreamer_pipeline_string()
        self.get_logger().info(f"Using GStreamer pipeline: {pipeline_str}")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name('appsink')

            if not self.pipeline:
                self.get_logger().error("Failed to create pipeline.")
                return
            if not self.appsink:
                self.get_logger().error("Failed to get appsink element.")
                return

            # Set properties on appsink
            self.appsink.set_property('emit-signals', True)
            self.appsink.set_property('max-buffers', 1) # Don't accumulate buffers
            self.appsink.set_property('drop', True) # Drop old buffers if queue is full

            # Connect the 'new-sample' signal
            self.appsink.connect('new-sample', self.on_new_sample)

            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error("Unable to set the pipeline to the playing state.")
                self.cleanup() # Clean up resources if start fails
                return
            elif ret == Gst.StateChangeReturn.ASYNC:
                 self.get_logger().warning("Pipeline starting asynchronously.")
                 # You might need to wait for state change here in more complex cases
            else:
                self.get_logger().info("Pipeline started successfully.")

        except Exception as e:
            self.get_logger().error(f"Error creating or starting GStreamer pipeline: {e}")
            self.cleanup()

    def on_new_sample(self, sink):
        sample = sink.pull_sample()
        if sample:
            try:
                # Get buffer and caps
                buf = sample.get_buffer()
                caps = sample.get_caps()
                
                # Extract image dimensions and format
                # Ensure caps are available and valid before accessing structure
                if not caps or not buf:
                    self.get_logger().warn("Could not get caps or buffer from sample.")
                    return Gst.FlowReturn.OK # Continue pipeline, drop frame

                structure = caps.get_structure(0)
                if not structure:
                    self.get_logger().warn("Could not get structure from caps.")
                    return Gst.FlowReturn.OK

                height = structure.get_value('height')
                width = structure.get_value('width')

                # Map buffer to readable memory
                success, map_info = buf.map(Gst.MapFlags.READ)
                if not success:
                    self.get_logger().warn("Failed to map buffer.")
                    return Gst.FlowReturn.OK

                # Create NumPy array from buffer data
                # Assuming BGR format from the pipeline
                frame = np.ndarray(
                    (height, width, 3), # Expecting 3 channels (BGR)
                    buffer=map_info.data,
                    dtype=np.uint8
                )

                # --- Compression ---
                encode_start_time = time.monotonic()
                result, compressed_data = cv2.imencode(f'.{self.format}', frame, self.imencode_param)
                encode_duration_ms = (time.monotonic() - encode_start_time) * 1000

                if not result:
                    self.get_logger().warn(f"Failed to encode frame to {self.format}")
                    buf.unmap(map_info) # MUST unmap the buffer
                    return Gst.FlowReturn.OK

                # --- Create and Publish ROS Message ---
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.format = self.format
                msg.data = compressed_data.tobytes() # Use tobytes() for >=numpy1.9

                self.publisher_.publish(msg)

                # --- Cleanup ---
                buf.unmap(map_info) # Unmap buffer IMPORTANT

            except Exception as e:
                 self.get_logger().error(f"Error processing frame: {e}")
                 # Try to unmap buffer if map_info exists
                 if 'map_info' in locals() and map_info is not None:
                     try:
                        buf.unmap(map_info)
                     except Exception as unmap_e:
                        self.get_logger().error(f"Error unmapping buffer during exception handling: {unmap_e}")

            finally:
                 # Regardless of success/failure in processing, return OK
                 # to allow GStreamer pipeline to continue
                 return Gst.FlowReturn.OK
        else:
             self.get_logger().warn("Appsink pull_sample returned None")
             return Gst.FlowReturn.ERROR # Indicate an issue getting the sample

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
        # Stop GLib Main Loop first
        if hasattr(self, 'mainloop') and self.mainloop.is_running():
            self.mainloop.quit()
        if hasattr(self, 'mainloop_thread') and self.mainloop_thread.is_alive():
            self.mainloop_thread.join(timeout=1.0) # Wait for thread to finish

        # Stop GStreamer pipeline
        if hasattr(self, 'pipeline') and self.pipeline:
            self.get_logger().info("Setting pipeline to NULL state.")
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None # Release reference

        self.get_logger().info("Cleanup complete.")


def main(args=None):
    rclpy.init(args=args)
    node = CsiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down.')
    finally:
        # Explicitly call cleanup before destroy_node
        node.cleanup()
        if rclpy.ok(): # Check if context is still valid
             node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown() # Shutdown RCLPY context

if __name__ == '__main__':
    main()