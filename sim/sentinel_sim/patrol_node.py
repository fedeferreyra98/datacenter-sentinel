"""
patrol_node.py
──────────────
Autonomous patrol controller for DataCenter Sentinel.

Reads waypoints from waypoints.yaml and sends sequential NavigateToPose
goals to Nav2. After each goal succeeds, publishes the rack_id on
/patrol/waypoint_reached so that virtual_sensor_node can take a reading.

Optionally loops the patrol (loop_patrol: true).

Topics published:
    /patrol/waypoint_reached  (std_msgs/String)  — rack_id on arrival
    /patrol/status            (std_msgs/String)  — JSON status for dashboard

Parameters:
    waypoints_file  (str)   Path to waypoints.yaml
    loop_patrol     (bool)  Loop indefinitely (default: false)
    use_sim_time    (bool)  Use /clock topic
"""

import json
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    """Convert yaw angle (rad) to quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop_patrol',    False)

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self._loop     = self.get_parameter('loop_patrol').get_parameter_value().bool_value

        # Load waypoints
        self._waypoints = self._load_waypoints(waypoints_file)
        if not self._waypoints:
            self.get_logger().error('No waypoints loaded — aborting.')
            raise RuntimeError('Empty waypoints list')

        self.get_logger().info(
            f'PatrolNode loaded {len(self._waypoints)} waypoints '
            f'(loop={self._loop})'
        )

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self._wp_pub     = self.create_publisher(String, '/patrol/waypoint_reached', 10)
        self._status_pub = self.create_publisher(String, '/patrol/status',           10)

        # Patrol state
        self._idx          = 0
        self._pass_count   = 0
        self._patrol_done  = False
        self._in_transit   = False
        self._start_ts     = time.monotonic()

        # Wait for Nav2 then begin
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready — starting patrol.')
        self._send_next_goal()

    # ── Waypoint loading ──────────────────────────────────────────────────

    def _load_waypoints(self, path: str) -> list:
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'Waypoints file not found: {path}')
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])

    # ── Navigation ────────────────────────────────────────────────────────

    def _send_next_goal(self):
        if self._patrol_done:
            return

        wp = self._waypoints[self._idx]
        rack_id = wp['rack_id']
        pose    = wp['pose']
        dwell   = wp.get('dwell_secs', 3.0)

        self.get_logger().info(
            f'→ Navigating to {rack_id} '
            f'(x={pose["x"]}, y={pose["y"]}, yaw={pose["yaw"]:.2f}rad)'
        )
        self._publish_status('navigating', rack_id)

        # Build PoseStamped goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(pose['x'])
        goal_msg.pose.pose.position.y = float(pose['y'])
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(pose['yaw']))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_accepted(f, rack_id, dwell)
        )

    def _on_goal_accepted(self, future, rack_id, dwell):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for {rack_id} — skipping.')
            self._advance()
            return

        self.get_logger().debug(f'Goal accepted for {rack_id}.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_goal_result(f, rack_id, dwell)
        )

    def _on_goal_result(self, future, rack_id, dwell):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✓ Arrived at {rack_id} — dwelling {dwell}s')
            self._publish_status('arrived', rack_id)

            # Notify sensor node
            msg = String()
            msg.data = rack_id
            self._wp_pub.publish(msg)

            # One-shot dwell timer.
            # create_timer() is repeating by default in ROS 2 — we cancel it
            # inside the callback so it fires exactly once.
            timer_ref = [None]

            def _dwell_fire():
                if timer_ref[0] is not None:
                    timer_ref[0].cancel()
                    timer_ref[0] = None
                    self._advance_once(rack_id)

            timer_ref[0] = self.create_timer(dwell, _dwell_fire)
        else:
            self.get_logger().warn(
                f'Navigation to {rack_id} ended with status {status} — advancing.'
            )
            self._advance()

    def _advance_once(self, rack_id):
        """Called once after dwell timer fires."""
        self.get_logger().info(f'Dwell complete at {rack_id}.')
        self._advance()

    def _advance(self):
        self._idx += 1
        if self._idx >= len(self._waypoints):
            self._pass_count += 1
            if self._loop:
                self.get_logger().info(
                    f'Pass #{self._pass_count} complete — restarting patrol.'
                )
                self._idx = 0
                self._send_next_goal()
            else:
                elapsed = time.monotonic() - self._start_ts
                self.get_logger().info(
                    f'Patrol complete — {len(self._waypoints)} racks visited '
                    f'in {elapsed:.1f}s.'
                )
                self._patrol_done = True
                self._publish_status('done', '')
        else:
            self._send_next_goal()

    # ── Feedback / status ─────────────────────────────────────────────────

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # Log distance remaining (throttled — only every ~2s in real usage)
        pass  # avoid log spam; uncomment below to debug
        # self.get_logger().debug(f'Distance remaining: {fb.distance_remaining:.2f}m')

    def _publish_status(self, state: str, rack_id: str):
        status = {
            'state':     state,
            'rack_id':   rack_id,
            'waypoint':  self._idx,
            'total':     len(self._waypoints),
            'pass':      self._pass_count,
            'elapsed_s': round(time.monotonic() - self._start_ts, 1),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
