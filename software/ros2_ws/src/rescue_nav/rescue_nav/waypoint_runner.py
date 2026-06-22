"""Drive the rescue track: visit waypoints (e.g. the end), then return to start.

A thin client of nav2_simple_commander's BasicNavigator. Defaults are tuned for
worlds/track.world (start near the origin facing +X, end deep in the L-branch),
but every pose is a ROS parameter so the same node works on a real saved map.

  ros2 run rescue_nav waypoint_runner
  ros2 run rescue_nav waypoint_runner --ros-args \
    -p waypoints:="[5.2, 2.6, 1.57,  0.3, 0.0, 3.14]" \
    -p set_initial_pose:=true -p initial_pose:="[0.0, 0.0, 0.0]"

`waypoints` is a flat [x, y, yaw, x, y, yaw, ...] list (metres, radians, map frame).
Progress is printed and republished on /nav/status (std_msgs/String) for the GUI.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _pose(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')
        # end of the L-branch, then back to the start (track.world defaults)
        # Defaults: drive to the far end of the straight corridor and back. These
        # are visible to the lidar from the start, so they work with online SLAM
        # (demo.launch.py). The L-branch goal (~5.2, 2.6) is occluded at startup and
        # needs the map-first workflow (build+save the full map, then localize).
        self.declare_parameter('waypoints', [4.5, 0.0, 0.0, 0.3, 0.0, 3.14])
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        # We localize with slam_toolbox (mapping/localization mode), not AMCL, and
        # slam_toolbox auto-activates without exposing a managed `get_state` service.
        # So wait on the navigator's lifecycle instead of a localizer's (this also
        # skips BasicNavigator's amcl-only initial-pose wait).
        self.declare_parameter('localizer', 'bt_navigator')

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/nav/status', latched)

    def publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRunner()
    navigator = BasicNavigator()

    flat = node.get_parameter('waypoints').value
    if len(flat) % 3 != 0 or len(flat) == 0:
        node.publish_status('aborted: waypoints must be a non-empty [x,y,yaw]*N list')
        navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        return
    goals = [_pose(navigator, flat[i], flat[i + 1], flat[i + 2])
             for i in range(0, len(flat), 3)]

    if node.get_parameter('set_initial_pose').value:
        ip = node.get_parameter('initial_pose').value
        navigator.setInitialPose(_pose(navigator, ip[0], ip[1], ip[2]))

    node.publish_status('waiting for Nav2 to activate...')
    navigator.waitUntilNav2Active(localizer=node.get_parameter('localizer').value)

    node.publish_status(f'navigating {len(goals)} waypoint(s)')
    navigator.followWaypoints(goals)

    while not navigator.isTaskComplete():
        rclpy.spin_once(node, timeout_sec=0.1)
        fb = navigator.getFeedback()
        if fb is not None:
            node.publish_status(
                f'navigating: at waypoint {fb.current_waypoint + 1}/{len(goals)}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.publish_status('reached: all waypoints complete')
    elif result == TaskResult.CANCELED:
        node.publish_status('aborted: navigation canceled')
    else:
        node.publish_status('aborted: navigation failed')

    navigator.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
