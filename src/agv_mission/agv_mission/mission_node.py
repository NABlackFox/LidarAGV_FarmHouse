#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger


class MissionNode(Node):

    def __init__(self):

        super().__init__('mission_node')

        self.get_logger().info('Starting mission node...')

        # Connect to Nav2    
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info(
            'Waiting for Nav2...'
        )

        self.nav_client.wait_for_server()

        self.get_logger().info(
            'Connected to Nav2'
        )

        # Camera service client
        self.camera_client = self.create_client(
            Trigger,
            '/top_camera/take_picture'
        )

        self.get_logger().info(
            'Waiting for camera service...'
        )

        self.camera_client.wait_for_service()

        self.get_logger().info(
            'Camera service connected'
        )

        # Load waypoint file
        self.load_waypoints()

        # Start mission
        self.run_mission()

    def load_waypoints(self):

        pkg_path = get_package_share_directory('agv_mission')

        yaml_file = os.path.join(pkg_path,'config','waypoints_map.yaml')

        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']

        self.get_logger().info(
            f'Loaded {len(self.points)} inspection points'
        )

    def create_goal(self, x, y, yaw):

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Position
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        # Orientation
        q = quaternion_from_euler(
            0.0,
            0.0,
            float(yaw)
        )

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal.pose = pose

        return goal

    def navigate_to(self, x, y, yaw):

        goal = self.create_goal(x, y, yaw)

        self.get_logger().info(
            f'Navigating to ({x:.2f}, {y:.2f})'
        )

        future = self.nav_client.send_goal_async(goal)

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

        result_future = (goal_handle.get_result_async())

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result.status == 4:

            self.get_logger().info(
                'Goal reached'
            )

            return True

        self.get_logger().warn(
            f'Navigation failed (status={result.status})'
        )

        return False

    def capture_image(self, plant_id):

        self.get_logger().info(
            f'Capturing image at {plant_id}'
        )

        request = Trigger.Request()

        future = self.camera_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        result = future.result()

        if result is None:

            self.get_logger().error(
                'Camera service call failed'
            )

            return False

        if result.success:

            self.get_logger().info(
                f'Image saved to: {result.message}'
            )

            return True

        self.get_logger().warn(
            f'Capture failed: {result.message}'
        )

        return False

    def run_mission(self):

        # Visit every plant
        for point in self.points:

            self.get_logger().info(
                f'Visiting {point["id"]}'
            )

            success = self.navigate_to(
                point['x'],
                point['y'],
                point['yaw']
            )

            if not success:

                self.get_logger().error(
                    'Mission aborted'
                )
                return

            self.capture_image(
                point['id']
            )

        # Return home
        self.get_logger().info(
            'Returning home'
        )

        success = self.navigate_to(
            self.home['x'],
            self.home['y'],
            self.home['yaw']
        )

        if success:

            self.get_logger().info(
                'Mission complete'
            )

        else:

            self.get_logger().error(
                'Failed to return home'
            )


def main(args=None):

    rclpy.init(args=args)
    node = MissionNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()