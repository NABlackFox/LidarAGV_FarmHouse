#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputeRoute, FollowPath
from custom_interfaces.srv import SetCameraState


class RouteTester(Node):

    def __init__(self):
        super().__init__('route_tester')

        # Action & Service Clients
        self.compute_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.camera_client = self.create_client(SetCameraState, '/camera/set_state')

        self.get_logger().info('Waiting for ROS 2 action servers and services...')
        self.compute_client.wait_for_server()
        self.follow_client.wait_for_server()
        self.camera_client.wait_for_service()
        self.get_logger().info('All servers and services connected!')

    # --- Camera Service ---------------------------------------------------
    def set_camera_state(self, state_str: str, table_id="TEST_POINT"):
        state_str = str(state_str).strip().lower()
        enable = (state_str in ['on', '1', 'true'])

        request = SetCameraState.Request()
        request.enable = enable
        request.table_id = str(table_id)

        future = self.camera_client.call_async(request)

        # Wait for service result without blocking the event loop
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'Camera set to {state_str.upper()} [ID: {table_id}]')
            return True
        else:
            self.get_logger().error(f'Failed to set camera state for {table_id}')
            return False

    # --- Routing & Navigation ---------------------------------------------
    def compute_route(self, start_id: int, goal_id: int):
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = start_id
        goal_msg.goal_id = goal_id
        goal_msg.use_start = True   # Use start_id instead of current pose
        goal_msg.use_poses = False  # Using graph node IDs

        send_future = self.compute_client.send_goal_async(goal_msg)

        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'ComputeRoute {start_id}->{goal_id} REJECTED!')
            return None

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result().result
        if result.error_code != 0:
            self.get_logger().error(f'ComputeRoute {start_id}->{goal_id} FAILED: {result.error_msg}')
            return None

        self.get_logger().info(f'Route computed successfully with {len(result.path.poses)} poses.')
        return result.path

    def follow_path(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        goal_msg.progress_checker_id = 'progress_checker'

        send_future = self.follow_client.send_goal_async(
            goal_msg, feedback_callback=self._follow_feedback_cb
        )

        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal REJECTED!')
            return False

        self.get_logger().info('FollowPath ACCEPTED. Driving...')
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            time.sleep(0.05)

        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Reached destination!')
            return True
        else:
            self.get_logger().error(f'FollowPath failed with status code {status}')
            return False

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {fb.distance_to_goal:.2f} m | Speed: {fb.speed:.2f} m/s',
            throttle_duration_sec=2.0
        )

    # --- Main Test Loop ----------------------------------------------------
    def interactive_loop(self):
        print("\n" + "=" * 50)
        print("         ROUTE MANUAL TESTER STARTED         ")
        print("=" * 50)

        while rclpy.ok():
            try:
                print("\nEnter test parameters (or type 'q' / 'exit' to quit):")

                start_input = input("  -> Start Node ID: ").strip()
                if start_input.lower() in ['q', 'exit']:
                    break
                start_id = int(start_input)

                end_input = input("  -> Goal Node ID: ").strip()
                if end_input.lower() in ['q', 'exit']:
                    break
                end_id = int(end_input)

                cam_input = input("  -> Camera state [on/off] (default: off): ").strip().lower()
                if not cam_input:
                    cam_input = "off"

                target_name = input("  -> Target Name/ID label (optional, default: TEST_PT): ").strip()
                if not target_name:
                    target_name = f"TEST_NODE_{end_id}"

                # --- Execution ---
                print(f"\n[EXEC] Testing Route {start_id} -> {end_id} | Camera: {cam_input.upper()}")

                # 1. Set Camera
                self.set_camera_state(cam_input, table_id=target_name)

                # 2. Compute Route
                path = self.compute_route(start_id, end_id)
                if path is None:
                    print("[ERROR] Route computation failed. Try another test.")
                    continue

                # 3. Follow Path
                success = self.follow_path(path)
                if success:
                    print(f"[SUCCESS] Route test {start_id} -> {end_id} COMPLETED!")
                else:
                    print(f"[ERROR] Navigation failed along segment {start_id} -> {end_id}.")

            except ValueError:
                print("[WARNING] Invalid input! Please enter integers for Node IDs.")
            except (KeyboardInterrupt, EOFError):
                break

        print("\nExiting Interactive Route Tester...")


def main(args=None):
    rclpy.init(args=args)
    node = RouteTester()

    # Spin ROS 2 callbacks in background thread so input() doesn't block ROS
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.interactive_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()