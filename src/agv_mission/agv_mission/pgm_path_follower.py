#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class PgmPathFollowerNode(Node):

    def __init__(self, navigator):
        # Set node name to pgm_path_follower
        super().__init__('pgm_path_follower')
        self.get_logger().info('Starting Dynamic PGM Path Follower Node...')

        self.navigator = navigator

        # Setup the camera service client
        self.camera_client = self.create_client(Trigger, '/top_camera/take_picture')
        self.get_logger().info('Waiting for camera service...')
        self.camera_client.wait_for_service()
        self.get_logger().info('Camera service connected')

        # Load map metadata and extract the path
        self.load_map_metadata_and_path()

    def load_map_metadata_and_path(self):
        pkg_path = get_package_share_directory('agv_mission')
        
        # 1. Load the metadata configuration updated to use your exact values
        # (Kept yaml parsing as fallback, but variables are explicitly hardcoded as requested)
        config_file = os.path.join(pkg_path, 'config', 'line_config.yaml')
        
        if os.path.exists(config_file):
            with open(config_file, 'r') as file:
                meta = yaml.safe_load(file)
            self.resolution = float(meta.get('resolution', 0.050))
            origin = meta.get('origin', [-13.086, -13.049, 0.0])
            self.origin_x = float(origin[0])
            self.origin_y = float(origin[1])
            self.home = meta.get('home', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        else:
            self.get_logger().warn(f"Config file not found at {config_file}. Using hardcoded fallback values.")
            # Your exact map parameters hardcoded straight into the execution variables
            self.resolution = 0.050
            self.origin_x = -13.086
            self.origin_y = -13.049
            self.home = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
        # 2. Extract the complete path line from your target image
        pgm_file_path = os.path.join(pkg_path, 'paths', 'path_line.pgm') 
        self.generated_path = self.extract_path_from_pgm(pgm_file_path)

    def extract_path_from_pgm(self, pgm_path):
        """Loads a PGM file, finds the starting pixel nearest to home, 
        and traces the entire path line from start to finish.
        """
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Failed to load path PGM image at: {pgm_path}")
            return None

        map_height_pixels = img.shape[0]
        
        # 1. Extract all black pixels
        black_pixels = np.argwhere(img < 50)
        if len(black_pixels) == 0:
            self.get_logger().error("No path line found in the PGM image!")
            return None

        # Convert to a set of tuples for efficient tracking lookups
        pixel_set = set(tuple(p) for p in black_pixels)

        # 2. Pinpoint the starting pixel based on the robot's 'home' position
        home_pixel_x = int((self.home['x'] - self.origin_x) / self.resolution)
        home_pixel_y = int(map_height_pixels - ((self.home['y'] - self.origin_y) / self.resolution))
        home_pixel = np.array([home_pixel_y, home_pixel_x])

        # Find the single black pixel closest to where the robot home is
        black_pixels_arr = np.array(list(pixel_set))
        distances_to_home = np.linalg.norm(black_pixels_arr - home_pixel, axis=1)
        closest_home_idx = np.argmin(distances_to_home)
        
        start_pt = tuple(black_pixels_arr[closest_home_idx])
        self.get_logger().info(f"Pinpointed start pixel at {start_pt} closest to home position.")

        # 3. Trace the entire loop using Nearest-Neighbor progression
        sorted_pixels = []
        current_pt = start_pt
        pixel_set.remove(current_pt)
        sorted_pixels.append(current_pt)

        search_radius = 3  # Maximum allowed step size between pixels along the line

        while pixel_set:
            remaining_arr = np.array(list(pixel_set))
            
            # Find the nearest remaining pixel along the drawn line
            distances = np.linalg.norm(remaining_arr - current_pt, axis=1)
            closest_idx = np.argmin(distances)
            
            # Gap checking safety (stops tracking if there is a break in the drawing)
            if distances[closest_idx] > (search_radius * 2):
                self.get_logger().warn("Reached the end of the line sequence or a physical gap.")
                break
                
            current_pt = tuple(remaining_arr[closest_idx])
            pixel_set.remove(current_pt)
            sorted_pixels.append(current_pt)

        # 4. Convert the full pixel sequence into ROS2 Map Coordinates
        nav2_path = Path()
        nav2_path.header.frame_id = 'map'
        nav2_path.header.stamp = self.navigator.get_clock().now().to_msg()

        for y_pixel, x_pixel in sorted_pixels:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = nav2_path.header.stamp
            
            world_x = self.origin_x + (x_pixel * self.resolution)
            world_y = self.origin_y + ((map_height_pixels - y_pixel) * self.resolution)
            
            pose.pose.position.x = float(world_x)
            pose.pose.position.y = float(world_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            nav2_path.poses.append(pose)

        self.get_logger().info(f"Successfully generated full path with {len(nav2_path.poses)} dense waypoints.")
        return nav2_path

    def execute_pgm_path_navigation(self):
        if self.generated_path is None or len(self.generated_path.poses) == 0:
            self.get_logger().error("Cannot navigate; loaded PGM path is empty or invalid.")
            return False

        # Attempt to smooth path via Simple Commander if supported
        smoothed_path = self.navigator.smoothPath(self.generated_path)
        if smoothed_path is None:
            self.get_logger().warn("Path smoothing failed, falling back to raw PGM pixel sequence.")
            # Fixed a potential reference issue by making sure fallback targets self.generated_path
            smoothed_path = self.generated_path

        self.get_logger().info('Injecting the path straight to the controller...')
        self.navigator.followPath(smoothed_path)

        # Execution monitoring loop
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'navigation_duration'):
                # Checks time duration (using basic float tracking)
                if feedback.navigation_duration > 600.0:
                    self.get_logger().error('Navigation timed out! Cancelling task...')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('End of PGM path line reached successfully!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Path follow operation was canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Path follow operation failed.')
        
        return False

    def capture_image(self):
        self.get_logger().info('Capturing image at inspection position')
        request = Trigger.Request()
        self.camera_client.call_async(request)
        return True

    def run_mission(self):
        self.get_logger().info('Starting continuous path execution...')
        success = self.execute_pgm_path_navigation()

        if not success:
            self.get_logger().error('Mission aborted due to path track failure')
            return

        # Trigger camera action at the final stop point
        self.capture_image()
        self.get_logger().info('Mission Complete.')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = BasicNavigator()
    print('Waiting for Nav2 to become fully active...')
    navigator.waitUntilNav2Active()
    print('Nav2 is active and ready!')

    node = PgmPathFollowerNode(navigator)
    
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()