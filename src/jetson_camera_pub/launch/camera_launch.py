import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

# --- Helper function to find the camera device ---
def find_camera_device(context, *args, **kwargs):
    camera_name_substring = LaunchConfiguration('camera_name').perform(context)
    default_device = LaunchConfiguration('default_device').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context) # Get node name for logging

    if not camera_name_substring:
        LogInfo(msg=f"[{node_name}] No camera_name specified, using default_device: {default_device}").execute(context)
        target_device = default_device
    else:
        LogInfo(msg=f"[{node_name}] Searching for camera device containing name: '{camera_name_substring}'").execute(context)
        try:
            # Run v4l2-ctl command
            result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=True, timeout=5)
            lines = result.stdout.splitlines()
            found_device = None
            for i, line in enumerate(lines):
                # Check if the camera name substring is in the current line
                if camera_name_substring in line:
                    # Check if the next line exists and looks like a device path
                    if i + 1 < len(lines) and lines[i+1].strip().startswith('/dev/video'):
                        found_device = lines[i+1].strip()
                        LogInfo(msg=f"[{node_name}] Found '{camera_name_substring}' at {found_device}").execute(context)
                        break # Found the first match

            if found_device:
                target_device = found_device
            else:
                LogInfo(msg=f"[{node_name}] Warning: Camera containing '{camera_name_substring}' not found. Falling back to default: {default_device}").execute(context)
                target_device = default_device

        except FileNotFoundError:
            LogInfo(msg=f"[{node_name}] Error: 'v4l2-ctl' command not found. Is v4l-utils installed? Falling back to default: {default_device}").execute(context)
            target_device = default_device
        except subprocess.TimeoutExpired:
             LogInfo(msg=f"[{node_name}] Error: 'v4l2-ctl --list-devices' timed out. Falling back to default: {default_device}").execute(context)
             target_device = default_device
        except subprocess.CalledProcessError as e:
            LogInfo(msg=f"[{node_name}] Error running v4l2-ctl: {e}. Falling back to default: {default_device}").execute(context)
            target_device = default_device
        except Exception as e:
            LogInfo(msg=f"[{node_name}] An unexpected error occurred during device search: {e}. Falling back to default: {default_device}").execute(context)
            target_device = default_device

    # --- Define the Node action here, after finding the device ---
    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    topic_name = LaunchConfiguration('topic_name').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    gst_pipeline_string = LaunchConfiguration('gst_pipeline_string').perform(context)


    camera_node = Node(
        package='jetson_camera_pub',
        executable='camera_node',
        name=node_name, # Use the node name passed via launch config
        output='screen',
        emulate_tty=True,
        parameters=[
            {'camera_device': target_device}, # Use the dynamically found or default device
            {'width': int(width)}, # Ensure correct type
            {'height': int(height)}, # Ensure correct type
            {'framerate': int(framerate)}, # Ensure correct type
            {'topic_name': topic_name},
            {'frame_id': frame_id},
            {'gst_pipeline_string': gst_pipeline_string}
        ]
    )
    # Return the node in a list for the launch system
    return [camera_node]

# --- Main launch description function ---
def generate_launch_description():

    # --- Declare arguments ---
    declare_camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        # IMPORTANT: Set the default value to a unique part of your desired camera's name
        default_value='HD Webcam C615',
        description='Substring of the target camera name to search for using v4l2-ctl. Leave empty to use default_device.'
    )

    declare_default_device_arg = DeclareLaunchArgument(
        'default_device',
        default_value='/dev/video1',
        description='Fallback device path if camera_name is empty or not found.'
    )

    declare_node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='gst_camera_publisher',
        description='Name for the ROS node.'
    )

    declare_width_arg = DeclareLaunchArgument(
        'width', default_value='640', description='Camera capture width.'
    )
    declare_height_arg = DeclareLaunchArgument(
        'height', default_value='480', description='Camera capture height.'
    )
    declare_framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='30', description='Camera capture framerate.'
    )
    declare_topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value='/camera/image_raw/compressed', description='Topic to publish compressed image.'
    )
    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='camera_link', description='Frame ID for the published message header.'
    )
    declare_gst_pipeline_arg = DeclareLaunchArgument(
        'gst_pipeline_string', default_value='', description='Optional: Override the entire GStreamer pipeline string.'
    )


    # --- Use OpaqueFunction to run Python code during launch ---
    # This ensures the device search happens *before* the Node action is fully configured
    create_camera_node = OpaqueFunction(function=find_camera_device)

    # --- Build the launch description ---
    ld = LaunchDescription()

    ld.add_action(declare_camera_name_arg)
    ld.add_action(declare_default_device_arg)
    ld.add_action(declare_node_name_arg)
    ld.add_action(declare_width_arg)
    ld.add_action(declare_height_arg)
    ld.add_action(declare_framerate_arg)
    ld.add_action(declare_topic_name_arg)
    ld.add_action(declare_frame_id_arg)
    ld.add_action(declare_gst_pipeline_arg)

    ld.add_action(create_camera_node) # This action will execute find_camera_device

    return ld