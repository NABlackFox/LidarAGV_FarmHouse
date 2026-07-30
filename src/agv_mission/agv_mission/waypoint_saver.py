#!/usr/bin/env python3
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Listens to RViz "Nav2 Goal" clicks
            self.listener_callback,
            10
        )
        self.waypoints = []
        self.count = 1
        self.get_logger().info('Click "2D Goal Pose" in RViz to add waypoints. Press Ctrl+C when done.')

    def listener_callback(self, msg):
        # Convert Quaternion to Yaw angle
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        point = {
            'id': f'point_{self.count}',
            'x': round(msg.pose.position.x, 2),
            'y': round(msg.pose.position.y, 2),
            'yaw': round(yaw, 4)
        }
        self.waypoints.append(point)
        self.get_logger().info(f'Added: {point["id"]} at x={point["x"]}, y={point["y"]}, yaw={point["yaw"]}')
        self.count += 1

    def save_to_yaml(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints captured, skipping save.')
            return
        
        # Target path relative to terminal execution directory
        file_path = Path('temp/waypoint_test.yaml')
        
        # Ensures the 'temp' folder exists inside your current terminal folder
        file_path.parent.mkdir(parents=True, exist_ok=True)

        # Format matching your mission node structure
        data = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'inspection_points': self.waypoints
        }
        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
        self.get_logger().info(f'Successfully saved {file_path.resolve()}!')

def main():
    rclpy.init()
    node = WaypointSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_yaml()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()