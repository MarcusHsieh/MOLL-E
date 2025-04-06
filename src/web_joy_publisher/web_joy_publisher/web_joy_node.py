import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import asyncio
import websockets
import json
import threading
import time

class WebJoyPublisherNode(Node):
    def __init__(self):
        super().__init__('web_joy_publisher_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)

        # --- Parameters ---
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('websocket_host', '0.0.0.0') # Listen on all interfaces
        # Define which Joy message axes correspond to inputs
        # Check 'ros2 topic echo /joy' from a real joystick to find typical indices
        # axes[0]: Left Stick X (unused here)
        # axes[1]: Left Stick Y (forward/backward)
        # axes[2]: Left Trigger (unused here)
        # axes[3]: Right Stick X (rotate left/right)
        # axes[4]: Right Stick Y (unused here)
        # axes[5]: Right Trigger (unused here)
        self.declare_parameter('axis_forward_backward', 1) # Index for left stick Y
        self.declare_parameter('axis_rotate_left_right', 3) # Index for right stick X
        self.declare_parameter('num_axes', 6) # Total number of axes in your Joy message
        self.declare_parameter('num_buttons', 0) # Total number of buttons

        # --- Internal State ---
        self._latest_left_y = 0.0
        self._latest_right_x = 0.0
        self._last_message_time = time.time()
        self._clients = set() # Keep track of connected clients

        # --- Start WebSocket server in a separate thread ---
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self._ws_thread.start()

        # --- Timer to publish Joy messages periodically ---
        self.timer = self.create_timer(0.05, self.publish_joy_message) # Publish at 20Hz

        self.get_logger().info('Web Joy Publisher Node started.')
        self.get_logger().info(f"Listening for WebSocket connections on port {self.get_parameter('websocket_port').value}")


    def _run_websocket_server(self):
        asyncio.set_event_loop(self._loop)
        port = self.get_parameter('websocket_port').value
        host = self.get_parameter('websocket_host').value

        try:
            start_server = websockets.serve(self._websocket_handler, host, port)
            self._loop.run_until_complete(start_server)
            self.get_logger().info(f"WebSocket server running on {host}:{port}")
            self._loop.run_forever()
        except OSError as e:
             self.get_logger().error(f"Failed to start WebSocket server on {host}:{port}: {e}")
             # Signal failure or attempt recovery? For now, just log.
        except Exception as e:
            self.get_logger().error(f"WebSocket server encountered an error: {e}")
        finally:
            self._loop.close()
            self.get_logger().info("WebSocket server stopped.")

    async def _websocket_handler(self, websocket, path):
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        self._clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self._latest_left_y = float(data.get('left_y', 0.0))
                    self._latest_right_x = float(data.get('right_x', 0.0))
                    self._last_message_time = time.time()
                    # Optional: Log received data for debugging
                    # self.get_logger().debug(f"Received: left_y={self._latest_left_y}, right_x={self._latest_right_x}")
                except json.JSONDecodeError:
                    self.get_logger().warn(f"Received invalid JSON from {websocket.remote_address}")
                except (ValueError, TypeError) as e:
                     self.get_logger().warn(f"Error processing data from {websocket.remote_address}: {e}")
                except Exception as e:
                    self.get_logger().error(f"Unexpected error processing message: {e}")

        except websockets.exceptions.ConnectionClosedOK:
            self.get_logger().info(f"Client disconnected gracefully: {websocket.remote_address}")
        except websockets.exceptions.ConnectionClosedError as e:
             self.get_logger().warn(f"Client connection closed with error: {websocket.remote_address} - {e}")
        finally:
            self._clients.remove(websocket)
            # If last client disconnects, maybe zero out inputs?
            if not self._clients:
                self._latest_left_y = 0.0
                self._latest_right_x = 0.0
                self.get_logger().info("Last client disconnected, zeroing inputs.")


    def publish_joy_message(self):
        # Optional: Add a timeout - if no message received for X seconds, zero out
        # timeout_duration = 1.0 # seconds
        # if time.time() - self._last_message_time > timeout_duration:
        #     self._latest_left_y = 0.0
        #     self._latest_right_x = 0.0
        #     if time.time() - self._last_message_time > timeout_duration + 0.1: # Log only once
        #         self.get_logger().warn("No recent web input, zeroing axes.")

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'web_joy' # Or just leave empty

        num_axes = self.get_parameter('num_axes').value
        num_buttons = self.get_parameter('num_buttons').value
        axis_fwd_idx = self.get_parameter('axis_forward_backward').value
        axis_rot_idx = self.get_parameter('axis_rotate_left_right').value

        msg.axes = [0.0] * num_axes
        msg.buttons = [0] * num_buttons

        # Clamp values just in case
        forward_val = max(-1.0, min(1.0, self._latest_left_y))
        rotate_val = max(-1.0, min(1.0, self._latest_right_x))

        if 0 <= axis_fwd_idx < num_axes:
            msg.axes[axis_fwd_idx] = forward_val
        else:
             self.get_logger().warn(f"Invalid axis index for forward/backward: {axis_fwd_idx}")

        if 0 <= axis_rot_idx < num_axes:
            msg.axes[axis_rot_idx] = rotate_val
        else:
             self.get_logger().warn(f"Invalid axis index for rotate: {axis_rot_idx}")

        self.publisher_.publish(msg)
        # self.get_logger().debug(f"Published Joy: Axes={msg.axes}") # Verbose debug


def main(args=None):
    rclpy.init(args=args)
    node = WebJoyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        # Node cleanup (publisher, timer automatically destroyed)
        # WebSocket server thread should exit due to daemon=True
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()