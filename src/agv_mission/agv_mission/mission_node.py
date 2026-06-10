#!/usr/bin/env python3

import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from ament_index_python.packages import get_package_share_directory


class MissionNode(Node):

    def __init__(self):

        super().__init__('mission_node')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.nav_client.wait_for_server()

        self.get_logger().info(
            'Connected to Nav2'
        )

        self.load_waypoints()

        self.run_mission()

    def load_waypoints(self):

        pkg_path = get_package_share_directory(
            'agv_mission'
        )

        yaml_file = (
            pkg_path +
            '/config/waypoints.yaml'
        )

        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']

    def create_goal(self, x, y):

        goal = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.w = 1.0

        goal.pose = pose

        return goal

    def navigate_to(self, x, y):

        goal = self.create_goal(x, y)

        self.get_logger().info(
            f'Navigating to ({x:.2f}, {y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Goal rejected'
            )

            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        self.get_logger().info(
            'Goal reached'
        )

        return True

    def capture_image(self, plant_id):

        self.get_logger().info(
            f'[CAMERA] Capturing image at {plant_id}'
        )

    def run_mission(self):

        for point in self.points:

            self.get_logger().info(
                f'Visiting {point["id"]}'
            )

            success = self.navigate_to(
                point['x'],
                point['y']
            )

            if not success:

                self.get_logger().error(
                    'Mission aborted'
                )

                return

            self.capture_image(
                point['id']
            )

        self.get_logger().info(
            'Returning home'
        )

        self.navigate_to(
            self.home['x'],
            self.home['y']
        )

        self.get_logger().info(
            'Mission Complete'
        )


def main(args=None):

    rclpy.init(args=args)

    node = MissionNode()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()