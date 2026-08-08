#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

# standard math approach to avoid requiring external 'transforms3d' library
import math

def euler_to_quaternion(yaw):
    """Convert a simple yaw angle (in radians) into a ROS 2 Quaternion."""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return [qw, qx, qy, qz]

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_init_amcl_pose')
        
        # Declare parameters with default values
        self.declare_parameter('x', -9.0)
        self.declare_parameter('y', -3.0)
        self.declare_parameter('theta', 0.1) 
        self.declare_parameter('cov', 0.5**2)
        
        # Create publisher
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait until AMCL is actually up and listening
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info("Waiting for AMCL Initial Pose subscriber...")
            time.sleep(1.0)
            
        # Trigger the message send
        self.send_init_pose()

    def send_init_pose(self):
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        theta = self.get_parameter('theta').value
        cov = self.get_parameter('cov').value
        
        msg = PoseWithCovarianceStamped()
        
        # Header configuration (with critical timestamp fix!)
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation using your yaw angle
        quat = euler_to_quaternion(theta)
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]
        
        # Covariance Matrix (Tells AMCL how certain we are)
        msg.pose.covariance = [
            cov, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos X
            0.0, cov, 0.0, 0.0, 0.0, 0.0,  # Pos Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot X
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot Y
            0.0, 0.0, 0.0, 0.0, 0.0, cov   # Rot Z
        ]
        
        self.get_logger().info(f"Publishing initial pose: x={x}, y={y}, theta={theta}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    # Spin once to ensure execution completes cleanly, then close down
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()