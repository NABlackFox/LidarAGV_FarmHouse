#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

# Custom Service import
from custom_interfaces.srv import SetCameraState


class PathFollow(Node):

    def __init__(self):
        super().__init__('path_follow')

        self.get_logger().info('Starting path follow mission node...')

        # Navigator
        self.navigator = BasicNavigator()

        # Camera state client
        self.camera_client = self.create_client(
            SetCameraState,
            '/camera/set_state'
        )

        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected!')

        # Load waypoints
        self.load_waypoints()

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 active!')

        self.current_start_pose = self.create_pose_stamped(
            self.home['x'], self.home['y'], self.home['yaw']
        )

        self.run_mission()

    def load_waypoints(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'config', 'waypoints_test.yaml')

        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']

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
        """Calls camera service with boolean enable and table_id string."""
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

    def follow_path_to_target(self, target_x, target_y, target_yaw, target_name="Target"):
        goal_pose = self.create_pose_stamped(target_x, target_y, target_yaw)

        path = self.navigator.getPath(self.current_start_pose, goal_pose)
        if not path or len(path.poses) == 0:
            self.get_logger().error(f'Failed to generate path to {target_name}!')
            return False

        smoothed_path = self.navigator.smoothPath(path)
        self.navigator.followPath(smoothed_path)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.current_start_pose = goal_pose
            return True

        return False

    def run_mission(self):
        # Initial state off
        self.set_camera_state('off', 'INITIAL')

        for point in self.points:
            point_id = point.get('id', 'Table_Unknown')
            camera_setting = point.get('camera', 'off')

            # Enable camera pass ID before traversing path
            if not self.set_camera_state(camera_setting, table_id=point_id):
                self.get_logger().error('Mission aborted due to camera service failure.')
                return

            success = self.follow_path_to_target(
                point['x'], point['y'], point['yaw'], target_name=point_id
            )

            if not success:
                self.get_logger().error('Mission aborted due to navigation failure.')
                self.set_camera_state('off', point_id)
                return

        # Turn camera off before going home
        self.set_camera_state('off', 'RETURNING_HOME')

        self.get_logger().info('Returning home...')
        self.follow_path_to_target(
            self.home['x'], self.home['y'], self.home['yaw'], target_name="Home"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollow()
    node.navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()