"""Launch the RoboCorea operator GUI (workstation).

Unlike the legacy stack there is NO gst_bridge node: the GUI pulls the C920 SRT
streams directly into its video widgets (OpenCV's GStreamer backend), so video
never makes a DDS round-trip. The robot-side streamer (scripts/c920_srt_stream.sh)
runs separately on the Jetson.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui',
            executable='gui',
            name='gui_node',
            output='screen',
        ),
    ])
