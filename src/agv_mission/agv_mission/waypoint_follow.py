#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

# Custom Service interface
from custom_interfaces.srv import SetCameraState


class WaypointFollow(Node):

    def __init__(self):
        super().__init__('waypoint_follow')

        self.get_logger().info('Starting modern waypoint follow mission node...')

        # Instantiate Nav2 Simple Commander Navigator (Modern API)
        self.navigator = BasicNavigator()

        # Camera state service client
        self.camera_client = self.create_client(
            SetCameraState,
            '/camera/set_state'
        )

        self.get_logger().info('Waiting for camera interval recording service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected!')

        # Load waypoints
        self.load_waypoints()

        # Wait for Nav2 stack to fully activate
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active and ready!')

        # Start mission
        self.run_mission()

    def load_waypoints(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'config', 'waypoints_test.yaml')

        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']

        self.get_logger().info(
            f'Loaded {len(self.points)} inspection points from waypoints_test.yaml'
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

    def set_camera_state(self, state_str, table_id="NONE"):
        """Calls camera service using clean spin_until_future_complete pattern."""
        state_str = str(state_str).lower()
        enable = (state_str == 'on')

        request = SetCameraState.Request()
        request.enable = enable
        request.table_id = str(table_id)

        self.get_logger().info(
            f'Setting camera state to {state_str.upper()} [ID: {table_id}]...'
        )

        future = self.camera_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and result.success:
            self.get_logger().info(
                f'Camera set to {state_str.upper()} [ID: {table_id}]'
            )
            return True
        else:
            self.get_logger().error(f'Failed to set camera state for {table_id}')
            return False

    def navigate_to_target(self, target_x, target_y, target_yaw, target_name="Target"):
        goal_pose = self.create_pose_stamped(target_x, target_y, target_yaw)

        self.get_logger().info(
            f'Navigating to [{target_name}] at ({target_x:.2f}, {target_y:.2f}, yaw={target_yaw:.2f})'
        )

        # Uses Nav2 Simple Commander goToPose
        self.navigator.goToPose(goal_pose)

        # Non-blocking check loop while navigation executes
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Successfully reached {target_name}!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'Navigation to {target_name} was canceled!')
        else:
            (error_code, error_msg) = self.navigator.getTaskError()
            self.get_logger().error(
                f'Navigation to {target_name} failed! Code {error_code}: {error_msg}'
            )

        return False

    def run_mission(self):
        # Initial camera state off
        self.set_camera_state('off', table_id='INITIAL')

        for point in self.points:
            point_id = point.get('id', 'Inspection Point')
            camera_setting = point.get('camera', 'off')

            self.get_logger().info(f'--- Starting leg for {point_id} ---')

            # Set camera state with ID before initiating movement
            if not self.set_camera_state(camera_setting, table_id=point_id):
                self.get_logger().error('Mission aborted due to camera service failure.')
                return

            # Navigate using goToPose
            success = self.navigate_to_target(
                point['x'], point['y'], point['yaw'], target_name=point_id
            )

            if not success:
                self.get_logger().error('Mission aborted due to navigation failure.')
                self.set_camera_state('off', table_id=point_id)
                return

        # Turn off camera before returning home
        self.set_camera_state('off', table_id='RETURNING_HOME')

        self.get_logger().info('Returning home...')
        success = self.navigate_to_target(
            self.home['x'], self.home['y'], self.home['yaw'], target_name="Home Base"
        )

        if success:
            self.get_logger().info('Mission complete!')
        else:
            self.get_logger().error('Failed to return home.')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollow()

    # Clean lifecycle shutdown of Nav2
    node.navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()