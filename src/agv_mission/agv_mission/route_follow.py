#!/usr/bin/env python3
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import ComputeRoute, FollowPath
from custom_interfaces.srv import SetCameraState


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower_node')

        self.compute_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')

        self.get_logger().info('Waiting for compute_route and follow_path servers...')
        self.compute_client.wait_for_server()
        self.follow_client.wait_for_server()
        self.get_logger().info('Servers connected!')

        self.camera_client = self.create_client(SetCameraState, '/camera/set_state')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected!')

        self.load_mission_routes()

    def load_mission_routes(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'config', 'mission_route.yaml')
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

    # --- Routing ------------------------------------------------------------
    def compute_route(self, start_id: int, goal_id: int):
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = start_id
        goal_msg.goal_id = goal_id
        goal_msg.use_start = True   # use start_id instead of TF current pose
        goal_msg.use_poses = False  # we're using graph node IDs, not poses

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
            self.get_logger().error(f'ComputeRoute {start_id}->{goal_id} failed: {result.error_msg}')
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
        status = result_future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Reached goal!')
            return True
        else:
            self.get_logger().error(f'FollowPath failed with status {status}')
            return False

    def follow_path_with_wait(self, path, retry_delay=3.0, max_retries=20):
        """Retries FollowPath if it aborts (e.g. progress-checker timeout while
        collision_monitor is holding the robot at a blocked path) instead of
        giving up on the mission."""
        attempts = 0
        while attempts < max_retries:
            if self.follow_path(path):
                return True
            attempts += 1
            self.get_logger().warn(
                f'FollowPath did not complete (attempt {attempts}/{max_retries}). '
                'Likely blocked — waiting and retrying.'
            )
            time.sleep(retry_delay)
        self.get_logger().error('FollowPath exceeded max retries. Giving up on this segment.')
        return False

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, speed: {fb.speed:.2f}',
            throttle_duration_sec=2.0
        )

    # --- Mission ------------------------------------------------------------
    def go_segment(self, start_id, end_id):
        path = self.compute_route(start_id, end_id)
        if path is None:
            return False
        return self.follow_path_with_wait(path)

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