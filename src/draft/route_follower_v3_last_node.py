#!/usr/bin/env python3
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputeRoute, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from custom_interfaces.srv import SetCameraState


# FollowPath (controller_server) result.error_code values.
# Source: nav2_msgs/action/FollowPath.action
FOLLOW_PATH_ERROR_CODES = {
    0: 'NONE',
    100: 'UNKNOWN',
    101: 'INVALID_CONTROLLER',
    102: 'TF_ERROR',
    103: 'INVALID_PATH',
    104: 'PATIENCE_EXCEEDED',
    105: 'FAILED_TO_MAKE_PROGRESS',
    106: 'NO_VALID_CONTROL',
}

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def describe_follow_path_error(code):
    name = FOLLOW_PATH_ERROR_CODES.get(code)
    if name is not None:
        return f'{name} ({code})'
    return f'UNRECOGNIZED_ERROR_CODE ({code}) -- look this up in nav2_msgs/action/FollowPath.action'


def describe_goal_status(status):
    name = GOAL_STATUS_NAMES.get(status)
    if name is not None:
        return f'{name} ({status})'
    return f'UNRECOGNIZED_GOAL_STATUS ({status})'


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower_node')

        self.compute_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')

        self.get_logger().info('Waiting for compute_route, follow_path servers...')
        self.compute_client.wait_for_server()
        self.follow_client.wait_for_server()
        self.get_logger().info('Servers connected!')

        self.camera_client = self.create_client(SetCameraState, '/camera/set_state')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected!')

        self.clear_local_client = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.clear_global_client = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self.get_logger().info('Waiting for costmap clearing services...')
        self.clear_local_client.wait_for_service()
        self.clear_global_client.wait_for_service()
        self.get_logger().info('Costmap clearing services connected!')

        self.load_mission_routes()

    def load_mission_routes(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'paths', 'mission_route.yaml')
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        self.home = data['home']
        self.points = data['inspection_points']

    # --- Camera -----------------------------------------------------------
    def set_camera_state(self, state_str, table_id="NONE"):
        state_str = str(state_str).lower()
        enable = (state_str == 'on')

        request = SetCameraState.Request()
        request.enable = enable
        request.table_id = str(table_id)

        future = self.camera_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'Camera set to {state_str.upper()} [ID: {table_id}]')
            return True
        else:
            self.get_logger().error(f'Failed setting camera state for {table_id}')
            return False

    # --- Recovery -----------------------------------------------------------
    def clear_costmaps(self):
        """Force-resets both costmaps to free space. Safe here because the
        environment is assumed static aside from people, who are the whole
        reason we stopped -- the very next scan re-marks anything still
        actually there within one cycle."""
        for name, client in (
            ('local', self.clear_local_client),
            ('global', self.clear_global_client),
        ):
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().warn(f'Failed to clear {name} costmap.')
        self.get_logger().info('Costmaps cleared.')

    # --- Routing ------------------------------------------------------------
    def get_node_sequence(self, start_id: int, goal_id: int):
        """Computes the route ONCE using the mission-defined start_id/goal_id
        (explicit, not live pose) purely to find out which graph nodes it
        passes through -- e.g. [0, 2, 3, 4, 5, 6, 7, 1]. This is the sparse
        `route.nodes` field, separate from the dense interpolated path."""
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = start_id
        goal_msg.goal_id = goal_id
        goal_msg.use_start = True    # use the given start_id, not live pose
        goal_msg.use_poses = False

        send_future = self.compute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'ComputeRoute {start_id}->{goal_id} rejected!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeRoute {start_id}->{goal_id} failed: {result.error_msg} '
                f'(error_code={result.error_code})'
            )
            return None

        node_ids = [n.nodeid for n in result.route.nodes]
        self.get_logger().info(f'Segment {start_id}->{goal_id} node sequence: {node_ids}')
        return node_ids

    def compute_hop_path(self, from_id: int, to_id: int):
        """Computes the dense drivable path for ONE hop between two
        explicit, known-valid graph nodes. No live pose involved -- both
        ends are always exact graph IDs, so there's no nearest-node
        snapping ambiguity to worry about."""
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = from_id
        goal_msg.goal_id = to_id
        goal_msg.use_start = True
        goal_msg.use_poses = False

        send_future = self.compute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'ComputeRoute {from_id}->{to_id} rejected!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeRoute {from_id}->{to_id} failed: {result.error_msg} '
                f'(error_code={result.error_code})'
            )
            return None

        return result.path

    def follow_path(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        goal_msg.progress_checker_id = 'progress_checker'

        send_future = self.follow_client.send_goal_async(
            goal_msg, feedback_callback=self._follow_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected!')
            return False

        self.get_logger().info('FollowPath ACCEPTED. Driving...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        status = wrapped_result.status
        error_code = wrapped_result.result.error_code

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached goal!')
            return True
        else:
            self.get_logger().error(
                f'FollowPath failed -- goal_status: {describe_goal_status(status)}, '
                f'error_code: {describe_follow_path_error(error_code)}'
            )
            return False

    def drive_hop(self, from_id, to_id, wait_before_clear=15.0, max_retries=20):
        """Drives ONE hop between two explicit graph nodes. On failure,
        waits, clears costmaps, and retries the SAME hop -- from_id never
        changes here, so 'resume from the last node visited' is exact and
        deterministic, not inferred from a possibly-noisy live pose."""
        attempts = 0
        while attempts < max_retries:
            path = self.compute_hop_path(from_id, to_id)
            if path is not None and self.follow_path(path):
                return True

            attempts += 1
            self.get_logger().warn(
                f'Hop {from_id}->{to_id} did not complete '
                f'(attempt {attempts}/{max_retries}). '
                f'Waiting {wait_before_clear:.0f}s in place, then clearing '
                f'costmaps and retrying from node {from_id} (last node visited).'
            )
            time.sleep(wait_before_clear)
            self.clear_costmaps()

        self.get_logger().error(f'Hop {from_id}->{to_id} exceeded max retries. Giving up.')
        return False

    def go_segment(self, start_id, end_id):
        """Breaks the segment into per-node hops and drives them in order,
        tracking the last node actually reached. A failure partway through
        only ever needs to resume from that last known node -- never from
        a live pose, never snapped from an arbitrary position."""
        node_ids = self.get_node_sequence(start_id, end_id)
        if not node_ids or len(node_ids) < 2:
            self.get_logger().error(f'Could not resolve a node path {start_id}->{end_id}.')
            return False

        current_node = node_ids[0]
        for next_node in node_ids[1:]:
            if not self.drive_hop(current_node, next_node):
                self.get_logger().error(
                    f'Segment {start_id}->{end_id} aborted at node {current_node} '
                    f'(last node successfully visited).'
                )
                return False
            current_node = next_node

        return True

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, speed: {fb.speed:.2f}',
            throttle_duration_sec=2.0
        )

    # --- Mission ------------------------------------------------------------
    def run_mission(self):
        self.set_camera_state('off', 'INITIAL')

        current_node = self.home['node_id']

        for point in self.points:
            point_id = point.get('id', 'Table_Unknown')
            camera_setting = point.get('camera', 'off')

            if not self.set_camera_state(camera_setting, table_id=point_id):
                self.get_logger().error('Mission aborted due to camera service failure.')
                return

            self.get_logger().info(f'--- Heading to {point_id} ---')
            if not self.go_segment(point['start_id'], point['end_id']):
                self.get_logger().error('Mission aborted due to navigation failure.')
                self.set_camera_state('off', point_id)
                return

            current_node = point['end_id']

        self.set_camera_state('off', 'RETURNING_HOME')
        self.get_logger().info('Returning home...')
        self.go_segment(current_node, self.home['node_id'])


def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
