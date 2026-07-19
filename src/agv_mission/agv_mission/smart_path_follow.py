#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class SmartPathFollowerNode(Node):

    def __init__(self, navigator):
        super().__init__('smart_path_follower')
        self.get_logger().info('Starting Hybrid PGM Waypoint Navigator Node...')

        self.navigator = navigator

        # Camera service
        self.camera_client = self.create_client(Trigger, '/top_camera/take_picture')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected')

        # Load configurations and map data
        self.load_map_and_extract_waypoints()

    def load_map_and_extract_waypoints(self):
        pkg_path = get_package_share_directory('agv_mission')
        
        # Exact hardcoded parameters as requested
        self.resolution = 0.050
        self.origin_x = -13.086
        self.origin_y = -13.049
        self.home = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Load the path image
        pgm_file_path = os.path.join(pkg_path, 'paths', 'path_line.pgm') 
        self.inspection_points = self.extract_waypoints_from_pgm(pgm_file_path)

    def extract_waypoints_from_pgm(self, pgm_path):
        """Loads the PGM path image and samples sparse waypoints every X pixels

        instead of feeding raw dense pixels, avoiding Nav2 controller crashes.
        """
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Failed to load path PGM image at: {pgm_path}")
            return []

        map_height_pixels = img.shape[0]
        black_pixels = np.argwhere(img < 50)
        
        if len(black_pixels) == 0:
            self.get_logger().error("No path line found in the PGM image!")
            return []

        # Sort pixels roughly by distance from home to maintain a sequence
        home_pixel_x = int((self.home['x'] - self.origin_x) / self.resolution)
        home_pixel_y = int(map_height_pixels - ((self.home['y'] - self.origin_y) / self.resolution))
        home_pixel = np.array([home_pixel_y, home_pixel_x])
        
        distances = np.linalg.norm(black_pixels - home_pixel, axis=1)
        sorted_indices = np.argsort(distances)
        sorted_black_pixels = black_pixels[sorted_indices]

        waypoints = []
        
        # STEP SIZE: Sample a waypoint every 20 pixels (~1 meter apart) 
        # This gives Nav2 a clear sequence without overwhelming the planner
        step_size = 20 
        
        for i in range(0, len(sorted_black_pixels), step_size):
            y_pixel, x_pixel = sorted_black_pixels[i]
            
            world_x = self.origin_x + (x_pixel * self.resolution)
            world_y = self.origin_y + ((map_height_pixels - y_pixel) * self.resolution)
            
            waypoints.append({
                'x': float(world_x),
                'y': float(world_y),
                'yaw': 0.0  # Default orientation forward
            })

        self.get_logger().info(f"Sampled {len(waypoints)} clean sparse waypoints from PGM map.")
        return waypoints

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

    def execute_navigation_to_waypoint(self, target_x, target_y, target_yaw):
        goal_pose = self.create_pose_msg(target_x, target_y, target_yaw)
        start_pose = PoseStamped()
        
        # Uses standard, robust global planning frame logic from Script 1
        initial_path = self.navigator.getPath(start_pose, goal_pose, use_start=False)
        if initial_path is None:
            self.get_logger().error("Global planner failed to find a valid path to waypoint!")
            return False

        smoothed_path = self.navigator.smoothPath(initial_path)
        if smoothed_path is None:
            smoothed_path = initial_path

        self.navigator.followPath(smoothed_path)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'navigation_duration'):
                if feedback.navigation_duration > 60.0: # Shorter timeout per waypoint
                    self.get_logger().error('Navigation timeout at waypoint!')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def capture_image(self):
        self.get_logger().info('Capturing inspection image...')
        request = Trigger.Request()
        self.camera_client.call_async(request)
        return True

    def run_mission(self):
        if not self.inspection_points:
            self.get_logger().error("No waypoints available. Aborting mission.")
            return

        self.get_logger().info('Executing mission across sampled PGM waypoints...')
        
        for idx, pt in enumerate(self.inspection_points):
            self.get_logger().info(f"Moving to path waypoint {idx+1}/{len(self.inspection_points)}")
            success = self.execute_navigation_to_waypoint(pt['x'], pt['y'], pt['yaw'])

            if not success:
                self.get_logger().warn(f"Failed to hit waypoint {idx+1}, skipping to maintain path sequence...")
                continue
            
            # Optional: take a picture at the very end or at certain intervals
            if idx == len(self.inspection_points) - 1:
                self.capture_image()

        # Return home safely
        self.get_logger().info('Returning home...')
        self.execute_navigation_to_waypoint(self.home['x'], self.home['y'], self.home['yaw'])
        self.get_logger().info('Mission Complete.')


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    node = SmartPathFollowerNode(navigator)
    
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()