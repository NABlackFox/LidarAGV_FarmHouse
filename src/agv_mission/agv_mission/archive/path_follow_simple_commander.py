#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class MissionNode(Node):

    def __init__(self, navigator):
        # Set node name to path_follow
        super().__init__('path_follow_simple_commander')
        self.get_logger().info('Starting High-Level Simple Commander Mission Node...')

        self.navigator = navigator

        # Setup the camera service client
        self.camera_client = self.create_client(Trigger, '/top_camera/take_picture')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected')

        self.load_waypoints()

    def load_waypoints(self):
        pkg_path = get_package_share_directory('agv_mission')
        yaml_file = os.path.join(pkg_path, 'config', 'waypoints_map.yaml')
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        self.home = data['home']
        self.points = data['inspection_points']
        self.get_logger().info(f'Loaded {len(self.points)} inspection points')

    def create_pose_msg(self, x, y, yaw):
        from tf_transformations import quaternion_from_euler
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def execute_path_navigation(self, target_x, target_y, target_yaw):
        goal_pose = self.create_pose_msg(target_x, target_y, target_yaw)

        # Pass empty placeholder for start; use_start=False auto-grabs the robot's real position
        start_pose = PoseStamped()
        self.get_logger().info("Requesting global plan...")
        initial_path = self.navigator.getPath(start_pose, goal_pose, use_start=False)
        
        if initial_path is None:
            self.get_logger().error("Global planner failed to find a valid path!")
            return False

        # Smooth path for clean driving on real AGVs
        smoothed_path = self.navigator.smoothPath(initial_path)
        if smoothed_path is None:
            self.get_logger().warn("Path smoothing failed, falling back to initial plan.")
            smoothed_path = initial_path

        self.get_logger().info(f'Following dense path directly to ({target_x:.2f}, {target_y:.2f})')
        
        # Call followPath without tracking a task handle variables
        self.navigator.followPath(smoothed_path)

        # Removed the 'task' argument entirely to match your environment's version
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'navigation_duration'):
                if feedback.navigation_duration > 600.0:
                    self.get_logger().error('Navigation timed out! Cancelling task...')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed to execute.')
        
        return False

    def capture_image(self, plant_id):
        self.get_logger().info(f'Capturing image at {plant_id}')
        request = Trigger.Request()
        self.camera_client.call_async(request)
        return True

    def run_mission(self):
        for point in self.points:
            self.get_logger().info(f'Visiting {point["id"]}')
            success = self.execute_path_navigation(point['x'], point['y'], point['yaw'])

            if not success:
                self.get_logger().error('Mission aborted due to navigation failure')
                return

            self.capture_image(point['id'])

        # Return home safely
        self.get_logger().info('Returning home...')
        self.execute_path_navigation(self.home['x'], self.home['y'], self.home['yaw'])
        self.get_logger().info('Mission Complete.')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = BasicNavigator()
    print('Waiting for Nav2 to become fully active...')
    navigator.waitUntilNav2Active()
    print('Nav2 is active and ready!')

    node = MissionNode(navigator)
    
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()