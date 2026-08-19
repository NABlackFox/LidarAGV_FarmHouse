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

# action_msgs/msg/GoalStatus values (the goal's overall status, separate
# from the controller's own error_code above).
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
    """Returns a readable name for a FollowPath error_code, or the raw
    code itself (clearly flagged) if it's not one we recognize -- so it's
    easy to look up in the Nav2 source/docs instead of guessing."""
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
    def compute_route(self, goal_id: int, expected_start_id: int = None):
        """Computes a route to goal_id. Uses the robot's LIVE pose as the
        start (use_start=False) so retries/restarts mid-segment work
        correctly, rather than forcing a fixed graph node as start.

        expected_start_id is only used for a sanity log -- it is NOT sent
        to the action. Keep it in your mission YAML to enforce/document
        segment ordering for whoever edits the mission later.
        """
        if expected_start_id is not None:
            self.get_logger().info(
                f'(Design intent: this segment expects to start near '
                f'node {expected_start_id} -> {goal_id})'
            )

        goal_msg = ComputeRoute.Goal()
        goal_msg.goal_id = goal_id
        goal_msg.use_start = False   # use robot's current pose, not a fixed node
        goal_msg.use_poses = False   # goal is still a graph node id

        send_future = self.compute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'ComputeRoute ->{goal_id} rejected!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeRoute ->{goal_id} failed: {result.error_msg} '
                f'(error_code={result.error_code})'
            )
            return None

        self.get_logger().info(f'Route computed with {len(result.path.poses)} poses.')
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

    def go_segment(self, start_id, end_id, wait_before_clear=15.0, max_retries=20):
        """Computes the route ONCE for this segment (from the robot's live
        pose, so a fresh call after a restart still resumes from wherever
        the robot actually is). On failure, retries by resending the SAME
        path rather than recomputing.

        Recomputing via ComputeRoute on every retry was the bug: the route
        server snaps to the nearest GRAPH NODE and re-solves from there --
        it does not plan from your exact live coordinate. If the direct
        edge you were on still scores as elevated cost near where you
        stopped, the graph solver can legitimately pick a totally
        different, disconnected route (e.g. looping around via the other
        lane) even though you're still sitting mid-path on a perfectly
        fine route. FollowPath's own controller already finds the closest
        point on a given path and resumes tracking from there -- that's
        exactly what we want for "paused mid-segment, now continue," so
        we let it do that instead of re-asking route_server each time.
        """
        path = self.compute_route(end_id, expected_start_id=start_id)
        if path is None:
            self.get_logger().error(f'Could not compute a route for segment ->{end_id}.')
            return False

        attempts = 0
        while attempts < max_retries:
            if self.follow_path(path):
                return True

            attempts += 1
            self.get_logger().warn(
                f'Segment ->{end_id} did not complete '
                f'(attempt {attempts}/{max_retries}). '
                f'Waiting {wait_before_clear:.0f}s in place, then clearing '
                'costmaps and resuming the same path.'
            )
            time.sleep(wait_before_clear)
            self.clear_costmaps()

        self.get_logger().error(f'Segment ->{end_id} exceeded max retries. Giving up.')
        return False

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