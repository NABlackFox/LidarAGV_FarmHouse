#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputeRoute, FollowPath


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower_node')

        self.compute_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')

        self.get_logger().info('Waiting for compute_route and follow_path servers...')
        self.compute_client.wait_for_server()
        self.follow_client.wait_for_server()
        self.get_logger().info('Servers connected!')

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
            self.get_logger().error('ComputeRoute goal rejected!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != 0:
            self.get_logger().error(f'ComputeRoute failed: {result.error_msg}')
            return None

        self.get_logger().info(f'Route computed with {len(result.path.poses)} poses.')
        return result.path

    def follow_path(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'          # matches your controller_server config
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
            self.get_logger().info('🎉 Reached goal!')
            return True
        else:
            self.get_logger().error(f'FollowPath failed with status {status}')
            return False

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, speed: {fb.speed:.2f}',
            throttle_duration_sec=2.0
        )

    def run_mission(self):
        self.get_logger().info('Computing full route 0 -> 7...')
        path = self.compute_route(start_id=0, goal_id=7)
        if path is None:
            self.get_logger().error('Could not compute route. Aborting.')
            return

        self.get_logger().info('Route ready. Handing off to controller_server...')
        self.follow_path(path)


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