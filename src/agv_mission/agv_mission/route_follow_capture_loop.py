#!/usr/bin/env python3
import math
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

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Frames used for the TF-based progress check below. Match these to
# your nav2_params.yaml (global_frame / robot_base_frame).
GLOBAL_FRAME = 'map'
ROBOT_FRAME = 'base_link'


# nav2_msgs/action/FollowPath.action result.error_code values
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

# action_msgs/msg/GoalStatus values
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
    return f'{name} ({code})' if name else f'UNKNOWN_ERROR_CODE ({code})'


def describe_goal_status(status):
    name = GOAL_STATUS_NAMES.get(status)
    return f'{name} ({status})' if name else f'UNKNOWN_GOAL_STATUS ({status})'


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower_node')

        # Nav2 action clients
        self.compute_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info('Waiting for compute_route, follow_path servers...')
        self.compute_client.wait_for_server()
        self.follow_client.wait_for_server()
        self.get_logger().info('Servers connected!')

        # Camera control service
        self.camera_client = self.create_client(SetCameraState, '/camera/set_state')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected!')

        # Costmap clearing services
        self.clear_local_client = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.clear_global_client = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self.get_logger().info('Waiting for costmap clearing services...')
        self.clear_local_client.wait_for_service()
        self.clear_global_client.wait_for_service()
        self.get_logger().info('Costmap clearing services connected!')

        # TF setup -- used by the watchdog to check real movement instead
        # of trusting FollowPath's own distance_to_goal feedback
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_distance_to_goal = None
        self.load_mission_routes()

    # --- TF progress check --------------------------------------------
    def get_current_position(self):
        """Looks up the robot's current (x, y) in GLOBAL_FRAME via TF.
        Returns None if the transform isn't available yet -- this is the
        ground-truth pose source, independent of FollowPath feedback."""
        try:
            tf = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y)

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

    # --- Costmap maintenance --------------------------------------------
    def clear_costmaps(self):
        """Resets local + global costmaps to free space."""
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
    def compute_route(self, start_id: int, goal_id: int):
        """Computes a route between two explicit graph nodes."""
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = start_id
        goal_msg.goal_id = goal_id
        goal_msg.use_start = True    # use start_id, not live pose
        goal_msg.use_poses = False

        # send the request and wait until it's accepted
        send_future = self.compute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'ComputeRoute {start_id}->{goal_id} rejected!')
            return None

        # wait for the actual route result to come back
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # check if route computation itself failed
        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeRoute {start_id}->{goal_id} failed: {result.error_msg} '
                f'(error_code={result.error_code})'
            )
            return None

        self.get_logger().info(f'Route computed with {len(result.path.poses)} poses.')
        return result.path

    def follow_path_persistent(self, path, stall_check_interval=15.0,
                                max_stalls=10, progress_epsilon=0.05):
        """Drives one path as a single FollowPath goal that survives
        stalls (e.g. person in the way). While stalled: clear costmaps
        and keep waiting, no new goal is created. Gives up only after
        max_stalls consecutive checks with no progress.

        Requires progress_checker.movement_time_allowance in
        nav2_params.yaml > stall_check_interval * max_stalls, otherwise
        controller_server aborts the goal before this watchdog runs.
        """
        # build the FollowPath goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        goal_msg.progress_checker_id = 'progress_checker'

        self._latest_distance_to_goal = None

        # send it and wait until it's accepted
        send_future = self.follow_client.send_goal_async(
            goal_msg, feedback_callback=self._follow_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected!')
            return False

        self.get_logger().info('FollowPath ACCEPTED. Driving...')
        result_future = goal_handle.get_result_async()

        last_seen_position = None
        stall_count = 0
        last_check_time = time.time()

        # main drive loop -- keeps spinning until the goal finishes
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)

            # timer check: only run the stall check every stall_check_interval seconds
            now = time.time()
            if now - last_check_time < stall_check_interval:
                continue
            last_check_time = now

            # progress check: real (x, y) displacement from TF, NOT
            # FollowPath's own distance_to_goal feedback -- that field
            # can freeze/jump on its own even while the robot is
            # actually driving fine, see notes on get_current_position.
            current_position = self.get_current_position()
            if current_position is None:
                continue  # TF not ready yet

            if last_seen_position is None:
                made_progress = True
            else:
                dx = current_position[0] - last_seen_position[0]
                dy = current_position[1] - last_seen_position[1]
                made_progress = math.hypot(dx, dy) > progress_epsilon
            last_seen_position = current_position

            if made_progress:
                if stall_count > 0:
                    self.get_logger().info('Progress resumed, stall counter reset.')
                stall_count = 0
                continue

            # no progress this interval -- bump stall counter and clear costmaps
            stall_count += 1
            logged_distance = (
                f'{self._latest_distance_to_goal:.2f} m'
                if self._latest_distance_to_goal is not None else 'unknown'
            )
            self.get_logger().warn(
                f'No progress for ~{stall_check_interval:.0f}s '
                f'(stall {stall_count}/{max_stalls}), '
                f'distance_to_goal={logged_distance} (feedback, informational only). '
                'Clearing costmaps, still waiting...'
            )
            self.clear_costmaps()

            # too many stalls in a row -- give up and cancel the goal
            if stall_count >= max_stalls:
                self.get_logger().error(
                    f'No progress after {max_stalls} checks '
                    f'(~{max_stalls * stall_check_interval:.0f}s). '
                    'Permanent blockage -- needs a human operator. Canceling.'
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        # goal finished (success, aborted, or canceled) -- check how it ended
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

    # feedback callback -- just caches the latest distance for the stall check above
    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._latest_distance_to_goal = fb.distance_to_goal
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, speed: {fb.speed:.2f}',
            throttle_duration_sec=2.0
        )

    # --- Mission ------------------------------------------------------------
    def go_segment(self, start_id, end_id):
        """Computes route between explicit graph nodes, drives it as a
        single persistent FollowPath goal, then clears costmaps on
        success (bumpy terrain blurs the map over a run)."""
        # step 1: compute the route
        path = self.compute_route(start_id, end_id)
        if path is None:
            self.get_logger().error(f'Could not compute a route {start_id}->{end_id}.')
            return False

        # step 2: drive the route
        success = self.follow_path_persistent(path)
        if success:
            self.get_logger().info(f'Segment {start_id}->{end_id} complete. Clearing costmaps.')
            self.clear_costmaps()
        return success

    def run_mission(self):
        # make sure camera starts OFF
        self.set_camera_state('off', 'INITIAL')

        current_node = self.home['node_id']

        # main loop over every inspection point in the yaml
        for point in self.points:
            point_id = point.get('id', 'Table_Unknown')
            camera_setting = point.get('camera', 'off')

            # set camera before driving to this point
            if not self.set_camera_state(camera_setting, table_id=point_id):
                self.get_logger().error('Mission aborted due to camera service failure.')
                return

            # drive to this point
            self.get_logger().info(f'--- Heading to {point_id} ---')
            if not self.go_segment(point['start_id'], point['end_id']):
                self.get_logger().error(
                    'Mission aborted -- segment failed permanently, needs human operator.'
                )
                self.set_camera_state('off', point_id)
                return

            current_node = point['end_id']

        # all points done -- head back home with camera off
        self.set_camera_state('off', 'RETURNING_HOME')
        self.get_logger().info('Returning home...')
        self.go_segment(current_node, self.home['node_id'])


class FollowRouteCaptureLoop(RouteFollower):

    def __init__(self):
        # keep everything from RouteFollower.__init__, just rename the node
        Node.__init__(self, 'follow_route_capture_loop')
        # re-run the rest of RouteFollower's setup manually since we
        # skipped RouteFollower.__init__ (it hardcodes its own node name)
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_distance_to_goal = None
        self.load_mission_routes()

        self.loop_count = 0

    def run_mission_loop(self):
        """Runs run_mission() (which already ends back at home) over and
        over, forever, with no gap in between. Prints/logs the current
        loop number each time a lap starts and finishes."""
        while rclpy.ok():
            self.loop_count += 1
            self.get_logger().info(f'=== Starting loop #{self.loop_count} ===')
            self.run_mission()
            self.get_logger().info(f'=== Finished loop #{self.loop_count} ===')


def main(args=None):
    rclpy.init(args=args)
    node = FollowRouteCaptureLoop()
    try:
        node.run_mission_loop()
    except KeyboardInterrupt:
        node.get_logger().info(f'Loop interrupted by user at loop #{node.loop_count}.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()