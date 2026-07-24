#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory


class MissionNode(Node):

    def __init__(self):
        super().__init__('path_follow')

        self.get_logger().info('Starting path follow mission node...')

        # Instantiate Nav2 Simple Commander Navigator
        self.navigator = BasicNavigator()

        # Load waypoint file
        self.load_waypoints()

        # Wait for Nav2 to activate
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active and ready!')

        # Set initial starting pose from home location
        self.current_start_pose = self.create_pose_stamped(
            self.home['x'], self.home['y'], self.home['yaw']
        )

        # Start mission
        self.run_mission()

    def load_waypoints(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'config', 'waypoints_map.yaml')

        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']

        self.get_logger().info(
            f'Loaded {len(self.points)} inspection points'
        )

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def follow_path_to_target(self, target_x, target_y, target_yaw, target_name="Target"):
        goal_pose = self.create_pose_stamped(target_x, target_y, target_yaw)

        # Print the upcoming destination
        self.get_logger().info(
            f'Upcoming path destination: [{target_name}] at ({target_x:.2f}, {target_y:.2f}, yaw={target_yaw:.2f})'
        )

        # 1. Generate path from current location to goal
        path = self.navigator.getPath(self.current_start_pose, goal_pose)

        if not path or len(path.poses) == 0:
            self.get_logger().error(f'Failed to generate path to {target_name}!')
            return False

        # 2. Smooth path
        smoothed_path = self.navigator.smoothPath(path)

        # 3. Follow the generated path
        self.get_logger().info(f'Executing path to {target_name}...')
        follow_task = self.navigator.followPath(smoothed_path)

        # Non-blocking spin loop while waiting for execution
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Successfully reached {target_name}!')
            # Update current pose for the next path start
            self.current_start_pose = goal_pose
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'Path to {target_name} was canceled!')
        else:
            (error_code, error_msg) = self.navigator.getTaskError()
            self.get_logger().error(f'Path execution failed! Code {error_code}: {error_msg}')

        return False

    def run_mission(self):
        # Visit every waypoint via path following
        for point in self.points:
            point_id = point.get('id', 'Inspection Point')

            success = self.follow_path_to_target(
                point['x'],
                point['y'],
                point['yaw'],
                target_name=point_id
            )

            if not success:
                self.get_logger().error('Mission aborted due to path following failure.')
                return

        # Return home
        self.get_logger().info('Returning home...')
        success = self.follow_path_to_target(
            self.home['x'],
            self.home['y'],
            self.home['yaw'],
            target_name="Home Base"
        )

        if success:
            self.get_logger().info('Mission complete!')
        else:
            self.get_logger().error('Failed to return home.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    # Clean shutdown of Nav2 lifecycle
    node.navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()