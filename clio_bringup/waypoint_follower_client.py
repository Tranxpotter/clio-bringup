"""
Waypoint Follower Client
This sends waypoints sequentially to Nav2's NavigateToPose action server,
with a configurable gap between each goal. 

Usage:
    ros2 run clio_bringup waypoint_follower_client
    ros2 run clio_bringup waypoint_follower_client --ros-args \
        -p poses_file:='/home/iwintern/guide_robot_ws/src/clio/clio_bringup/config/waypoints_example.yaml' \
        -p goal_delay:=2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import yaml
import time


def create_pose_stamped(node: Node, x: float, y: float, yaw: float, frame_id: str = "map") -> PoseStamped:
    """Create a PoseStamped message from x, y, and yaw (in radians)."""
    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion (roll=pitch=0)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    pose.pose.orientation.w = cy
    pose.pose.orientation.z = sy
    
    return pose


class WaypointFollowerClient(Node):
    """Client that sends waypoints to Nav2's NavigateToPose action server."""
    
    def __init__(self):
        super().__init__('waypoint_follower_client')
        
        # Declare parameters
        self.declare_parameter('poses_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_delay', 0.5)  # Delay in seconds between goals
        self.declare_parameter('autonomous_back', False)
        
        self.poses_file = self.get_parameter('poses_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.goal_delay = self.get_parameter('goal_delay').get_parameter_value().double_value
        self.autonomous_back = self.get_parameter('autonomous_back').get_parameter_value().bool_value
        
        # Create action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # State variables for sequential waypoint navigation
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # Load waypoints
        self.waypoints = self._load_waypoints()
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"Goal delay set to {self.goal_delay} seconds")
        
    def _load_waypoints(self) -> list:
        """Load waypoints from file or return default demo waypoints."""
        if self.poses_file:
            try:
                with open(self.poses_file, 'r') as f:
                    data = yaml.safe_load(f)
                    return data.get('waypoints', [])
            except Exception as e:
                self.get_logger().error(f"Failed to load poses file: {e}")
                return []
        else:
            self.get_logger().error("No poses file provided!")
            return []
    
    def start_navigation(self):
        """Start navigating to the first waypoint."""
        if not self.waypoints:
            self.get_logger().error("No waypoints to navigate to!")
            rclpy.shutdown()
            return
        
        # Wait for action server
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            rclpy.shutdown()
            return
        
        self.current_waypoint_index = 0
        self._send_next_goal()
    
    def _send_next_goal(self):
        """Send the next waypoint as a goal."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("✓ All waypoints completed successfully!")
            rclpy.shutdown()
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        x, y, yaw_deg = waypoint[0], waypoint[1], waypoint[2]
        yaw_rad = math.radians(yaw_deg)
        
        goal_pose = create_pose_stamped(self, x, y, yaw_rad, self.frame_id)
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(
            f"[{self.current_waypoint_index + 1}/{len(self.waypoints)}] "
            f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°"
        )
        
        # Send goal
        self.is_navigating = True
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal {self.current_waypoint_index + 1} rejected :(')
            self.is_navigating = False
            # Try to continue with next goal
            self._prepare_next_goal()
            return
        
        self.get_logger().info(f'Goal {self.current_waypoint_index + 1} accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        # self.get_logger().debug(f"Distance remaining: {feedback.distance_remaining:.2f}m")
    
    def _get_result_callback(self, future):
        """Handle result when navigation to current waypoint completes."""
        result = future.result()
        self.is_navigating = False
        
        if result.status == 4:  # STATUS_SUCCEEDED = 4
            self.get_logger().info(f"Goal {self.current_waypoint_index + 1}/{len(self.waypoints)} reached!")
        else:
            self.get_logger().warning(f"Goal {self.current_waypoint_index + 1} failed with status: {result.status}")
        
        # Move to next waypoint
        self._prepare_next_goal()
    
    def _prepare_next_goal(self):
        """Prepare to send the next goal after a delay."""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("✓ All waypoints completed!")
            rclpy.shutdown()
            return
        
        # Apply delay before sending next goal
        if self.goal_delay > 0:
            self.get_logger().info(f"Waiting {self.goal_delay} seconds before next goal...")
            # Use create_timer for non-blocking delay
            self._delay_timer = self.create_timer(self.goal_delay, self._delay_timer_callback)
        else:
            self._send_next_goal()
    
    def _delay_timer_callback(self):
        """Called after the delay timer expires."""
        # Cancel and destroy the timer
        self._delay_timer.cancel()
        self.destroy_timer(self._delay_timer)
        
        # Send the next goal
        self._send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    
    client = WaypointFollowerClient()
    client.start_navigation()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info("Interrupted by user")
    finally:
        client.destroy_node()


if __name__ == '__main__':
    main()
