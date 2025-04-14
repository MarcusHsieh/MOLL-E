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
import threading # Added for mainloop thread

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GstCameraNode(Node):
    def __init__(self):
        super().__init__('gst_camera_node') # Node name used internally

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
        try:
            Gst.init(sys.argv)
        except Exception as e:
             self.get_logger().error(f"Error initializing GStreamer: {e}")
             rclpy.shutdown()
             sys.exit(1)

        self.pipeline = None
        self.appsink = None
        self.mainloop = GLib.MainLoop()
        self.mainloop_thread = None # Initialize thread attribute

        # Handle Ctrl+C
        # Use a lambda to pass 'self' to the handler if needed, or keep it simple
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler) # Handle termination signal too

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
            # Ensure cleanup happens even if parsing fails
            self.cleanup()
            rclpy.shutdown()
            sys.exit(1)
        except Exception as e: # Catch other potential parsing errors
            self.get_logger().error(f"An unexpected error occurred during pipeline parsing: {e}")
            self.cleanup()
            rclpy.shutdown()
            sys.exit(1)


        # Get the appsink element
        self.appsink = self.pipeline.get_by_name('ros_sink')
        if not self.appsink:
            self.get_logger().error("Could not find appsink element 'ros_sink' in the pipeline.")
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            self.cleanup() # Ensure cleanup
            rclpy.shutdown()
            sys.exit(1)

        # Set the callback for new samples
        self.appsink.set_property('emit-signals', True)
        # Use connect_after for potentially safer signal handling in threaded contexts
        self.appsink.connect('new-sample', self.on_new_sample)

    def start_pipeline(self):
        if self.pipeline:
            self.get_logger().info("Setting pipeline to PLAYING...")
            ret = self.pipeline.set_state(Gst.State.PLAYING)

            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error("Unable to set the pipeline to the playing state.")
                # Attempt to get more details from the bus
                bus = self.pipeline.get_bus()
                if bus:
                    msg = bus.timed_pop_filtered(
                        Gst.SECOND * 1, # Wait up to 1 second
                        Gst.MessageType.ERROR | Gst.MessageType.WARNING
                    )
                    if msg:
                         err, debug_info = msg.parse_error()
                         self.get_logger().error(f"GStreamer Error: {err}, Debug Info: {debug_info}")
                self.cleanup()
                # Use rclpy shutdown mechanism properly
                if rclpy.ok():
                    rclpy.shutdown()
                sys.exit(1) # Exit after shutdown attempt

            elif ret == Gst.StateChangeReturn.ASYNC:
                 self.get_logger().info("Pipeline state change is asynchronous. Waiting for ASYNC_DONE...")
                 # Wait for the state change to complete
                 ret, state, pending = self.pipeline.get_state(Gst.CLOCK_TIME_NONE) # Block until state change is done
                 if ret == Gst.StateChangeReturn.FAILURE:
                     self.get_logger().error("Pipeline failed to reach PLAYING state asynchronously.")
                     self.cleanup()
                     if rclpy.ok():
                         rclpy.shutdown()
                     sys.exit(1)
                 elif state == Gst.State.PLAYING:
                     self.get_logger().info("Pipeline state successfully set to PLAYING (async).")
                 else:
                      self.get_logger().warning(f"Pipeline ended up in state {state.value_nick} after async change.")


            elif ret == Gst.StateChangeReturn.NO_PREROLL:
                 self.get_logger().warning("Pipeline is live and does not need preroll. State is PLAYING.")
            else:
                 self.get_logger().info("Pipeline state set to PLAYING.")

            # Start the GLib main loop in a separate thread only if pipeline started successfully
            # Check if thread already exists and is running - might happen on restarts? (unlikely with current structure)
            if self.mainloop_thread is None or not self.mainloop_thread.is_alive():
                self.mainloop_thread = threading.Thread(target=self.mainloop.run, daemon=True) # Use daemon thread
                self.mainloop_thread.start()
                self.get_logger().info("GStreamer main loop started.")
            else:
                self.get_logger().warning("Main loop thread already running?")

        else:
            self.get_logger().error("Pipeline not initialized. Cannot start.")

    # ============================================
    # MODIFIED on_new_sample function (using emit)
    # ============================================
    def on_new_sample(self, appsink):
        """Callback function called when a new sample is available."""

        # --- TEMPORARY DEBUGGING (Optional - Can remove later) ---
        try:
            # Can remove this section once it's working
            if not hasattr(self, '_debug_printed'): # Print only once
                 self.get_logger().info("--- Appsink available methods/attributes (Check for Emit): ---")
                 methods = dir(appsink)
                 for i in range(0, len(methods), 5): self.get_logger().info(f"{methods[i:i+5]}")
                 self.get_logger().info("--- End of Appsink methods/attributes ---")
                 self.get_logger().info(f"Has 'try_pull_sample'? {hasattr(appsink, 'try_pull_sample')}")
                 self.get_logger().info(f"Has 'pull_sample'? {hasattr(appsink, 'pull_sample')}")
                 self.get_logger().info(f"Has 'emit'? {hasattr(appsink, 'emit')}")
                 self._debug_printed = True # Flag to prevent re-printing
        except Exception as e:
            self.get_logger().error(f"Error during debug printing: {e}")
        # --- END OF TEMPORARY DEBUGGING ---

        sample = None # Initialize sample to None

        # --- Try using emit instead of direct attribute access ---
        try:
            # self.get_logger().debug("Attempting to emit 'try-pull-sample'...")
            # Emit the "try-pull-sample" action signal.
            # Argument 1: signal name (string)
            # Argument 2: timeout in nanoseconds (Gst.ClockTime). 0 = non-blocking.
            # The return value of emit for action signals is the return value of the action.
            sample = appsink.emit("try-pull-sample", 0)
            # self.get_logger().debug(f"Result of emit('try-pull-sample', 0): {type(sample)}")

        except GLib.Error as emit_err:
             # Catch potential errors if the action signal itself doesn't exist or fails
             self.get_logger().error(f"GLib.Error emitting 'try-pull-sample': {emit_err}", throttle_duration_sec=5)
             return Gst.FlowReturn.OK # Allow pipeline to continue
        except TypeError as emit_type_err:
             # Catch error if 'emit' exists but signal name is wrong or args don't match
             self.get_logger().error(f"TypeError emitting 'try-pull-sample': {emit_type_err}. Check signal name and arguments.", throttle_duration_sec=5)
             return Gst.FlowReturn.OK
        except Exception as emit_exc:
             self.get_logger().error(f"Unexpected Error emitting 'try-pull-sample': {emit_exc}", throttle_duration_sec=5)
             return Gst.FlowReturn.OK # Allow pipeline to continue
        # --- End of emit attempt ---


        # Check if the emit call succeeded in returning a valid sample object
        # Note: Gst.Sample is the expected type if successful
        if sample is None or not isinstance(sample, Gst.Sample):
            # This can happen if timeout occurs (with non-zero timeout) or if emit failed subtly.
            self.get_logger().warn(f"emit('try-pull-sample', 0) returned {type(sample)} instead of Gst.Sample.", throttle_duration_sec=5)
            return Gst.FlowReturn.OK # Still signal GStreamer to continue

        # --- REST OF THE FUNCTION (buffer processing, publishing) REMAINS THE SAME ---
        try:
            buffer = sample.get_buffer()
            # Make sure buffer is valid before proceeding
            if buffer is None:
                 self.get_logger().warn("Sample contained no buffer.", throttle_duration_sec=5)
                 return Gst.FlowReturn.OK

            # Map the buffer for reading
            result, map_info = buffer.map(Gst.MapFlags.READ)

            if not result:
                 self.get_logger().error("Failed to map GStreamer buffer for reading.", throttle_duration_sec=5)
                 # Unmap might not be needed if map failed, but doesn't hurt to try in finally
                 return Gst.FlowReturn.OK

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = map_info.data

            self.publisher_.publish(msg)

        except GLib.Error as e:
            self.get_logger().error(f"GLib Error processing GStreamer sample: {e}", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error processing GStreamer sample: {e}", throttle_duration_sec=5)
        finally:
            # Unmap the buffer if it was successfully mapped
            if 'map_info' in locals() and map_info is not None:
                 buffer.unmap(map_info)
            # Sample reference handling is typically managed by GStreamer/GI here

        # Indicate success to GStreamer
        return Gst.FlowReturn.OK
    # ============================================
    # End of MODIFIED on_new_sample function
    # ============================================


    def cleanup(self):
        self.get_logger().info("Initiating cleanup...")

        # Stop the main loop first
        if hasattr(self, 'mainloop') and self.mainloop is not None and self.mainloop.is_running():
            self.get_logger().info("Quitting GStreamer main loop...")
            self.mainloop.quit()

        # Wait for the main loop thread to finish
        if hasattr(self, 'mainloop_thread') and self.mainloop_thread is not None and self.mainloop_thread.is_alive():
             self.get_logger().info("Joining GStreamer main loop thread...")
             self.mainloop_thread.join(timeout=2) # Wait max 2 seconds
             if self.mainloop_thread.is_alive():
                 self.get_logger().warning("Mainloop thread did not exit cleanly after 2 seconds.")
             else:
                 self.get_logger().info("Main loop thread joined.")
        self.mainloop_thread = None # Clear thread reference

        # Set pipeline to NULL state
        if hasattr(self, 'pipeline') and self.pipeline is not None:
            self.get_logger().info("Setting GStreamer pipeline to NULL state...")
            self.pipeline.set_state(Gst.State.NULL)
            self.get_logger().info("Pipeline state set to NULL.")
            # Clear references - helps GC but Gst should manage internal refs too
            self.pipeline = None
            self.appsink = None

        # No explicit Gst.deinit() needed usually when script exits

        self.get_logger().info("Cleanup complete.")

    def signal_handler(self, sig, frame):
        # This handler is called by the OS signal
        self.get_logger().info(f'Signal {sig} received. Shutting down node.')
        # This cleanup call will handle GStreamer shutdown
        self.cleanup()
        # Initiate ROS shutdown AFTER internal cleanup if possible
        # Check if context is still valid before calling shutdown
        if rclpy.ok():
            rclpy.shutdown()
        # sys.exit(0) # Often not needed if rclpy.spin() returns properly


def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        node = GstCameraNode()
        node.start_pipeline()
        if node.pipeline and node.appsink: # Only spin if pipeline started okay
             rclpy.spin(node)
        else:
             node.get_logger().error("Node initialization or pipeline start failed. Not spinning.")
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt caught, initiating shutdown.')
        else:
            print('KeyboardInterrupt caught during node initialization.')
    except Exception as e:
        # Log exceptions during node initialization or spin
        if node:
             node.get_logger().fatal(f"Unhandled exception in main spin loop: {e}", )
             import traceback
             node.get_logger().error(traceback.format_exc()) # Log stack trace
        else:
             print(f"Unhandled exception during node initialization: {e}")
             import traceback
             traceback.print_exc()
    finally:
        # This cleanup runs regardless of how the try block exits
        if node is not None:
            node.get_logger().info("Performing final cleanup...")
            node.cleanup() # Ensure GStreamer is stopped

        # Ensure ROS is shut down if it was initialized and is still running
        if rclpy.ok():
            rclpy.shutdown()
            print("rclpy shutdown complete.")
        else:
             print("rclpy already shut down or not initialized.")

if __name__ == '__main__':
    main()