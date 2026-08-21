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

        self._latest_distance_to_goal = None
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
        """Force-resets both costmaps to free space, WITHOUT touching any
        in-progress FollowPath goal -- this is a separate service call, so
        it's safe to fire while the robot is mid-action and stalled."""
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
        """Computes a route to goal_id from the robot's LIVE pose. Called
        ONCE per segment (not on every stall) -- see follow_path_persistent,
        which keeps a single goal running through stalls instead of
        recomputing."""
        if expected_start_id is not None:
            self.get_logger().info(
                f'(Design intent: this segment expects to start near '
                f'node {expected_start_id} -> {goal_id})'
            )

        goal_msg = ComputeRoute.Goal()
        goal_msg.goal_id = goal_id
        goal_msg.use_start = False   # robot's current pose, not a fixed node
        goal_msg.use_poses = False

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

    def follow_path_persistent(self, path, stall_check_interval=15.0,
                                max_stalls=20, progress_epsilon=0.05):
        """Sends FollowPath ONCE and keeps that SAME goal running through
        obstacle stalls, instead of letting it abort and creating a new
        goal on every retry (that's what caused the MPPI cold-start /
        INVALID_PATH problem previously). While distance_to_goal isn't
        improving, we periodically clear costmaps in the background --
        without touching the goal at all -- and just keep waiting. Only
        after max_stalls consecutive checks with no real progress do we
        give up and treat it as a permanent blockage.

        IMPORTANT: requires progress_checker.movement_time_allowance in
        nav2_params.yaml to be raised well above
        stall_check_interval * max_stalls, otherwise controller_server
        aborts the goal on its own before this watchdog ever runs.
        """
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        goal_msg.progress_checker_id = 'progress_checker'

        self._latest_distance_to_goal = None

        send_future = self.follow_client.send_goal_async(
            goal_msg, feedback_callback=self._follow_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected!')
            return False

        self.get_logger().info('FollowPath ACCEPTED. Driving...')
        result_future = goal_handle.get_result_async()

        last_seen_distance = None
        stall_count = 0
        last_check_time = time.time()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)

            now = time.time()
            if now - last_check_time < stall_check_interval:
                continue
            last_check_time = now

            current_distance = self._latest_distance_to_goal
            if current_distance is None:
                continue  # no feedback received yet

            made_progress = (
                last_seen_distance is None
                or (last_seen_distance - current_distance) > progress_epsilon
            )
            last_seen_distance = current_distance

            if made_progress:
                if stall_count > 0:
                    self.get_logger().info('Progress resumed, stall counter reset.')
                stall_count = 0
                continue

            stall_count += 1
            self.get_logger().warn(
                f'No progress for ~{stall_check_interval:.0f}s '
                f'(stall {stall_count}/{max_stalls}), '
                f'distance_to_goal={current_distance:.2f} m. '
                'Clearing costmaps, still waiting...'
            )
            self.clear_costmaps()

            if stall_count >= max_stalls:
                self.get_logger().error(
                    f'No progress after {max_stalls} checks '
                    f'(~{max_stalls * stall_check_interval:.0f}s total). '
                    'Treating as a permanent blockage that needs a human. '
                    'Canceling goal and stopping mission.'
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        # Goal ended on its own (success, or a real controller-level
        # failure like TF_ERROR/INVALID_PATH -- not a stall we're handling).
        wrapped_result = result_future.result()
        status = wrapped_result.status
        error_code = wrapped_result.result.error_code

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached goal!')
            return True
        else:
            self.get_logger().error(
                f'FollowPath ended -- goal_status: {describe_goal_status(status)}, '
                f'error_code: {describe_follow_path_error(error_code)}'
            )
            return False

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._latest_distance_to_goal = fb.distance_to_goal
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, speed: {fb.speed:.2f}',
            throttle_duration_sec=2.0
        )

    # --- Mission ------------------------------------------------------------
    def go_segment(self, start_id, end_id):
        """Computes the route ONCE (from live pose, so a mission restart
        still resumes from wherever the robot actually is), then drives it
        as a single persistent FollowPath goal that survives stalls."""
        path = self.compute_route(end_id, expected_start_id=start_id)
        if path is None:
            self.get_logger().error(f'Could not compute a route for segment ->{end_id}.')
            return False
        return self.follow_path_persistent(path)

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
                self.get_logger().error(
                    'Mission aborted -- segment failed permanently, needs human operator.'
                )
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
