#!/usr/bin/env python3
import rclpy
import csv
import os
import signal
from datetime import datetime
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize parameters
        self.declare_parameter('retry_attempts', 3)
        self.declare_parameter('tf_timeout', 1.0)
        
        # Configure waypoints
        self.waypoints = [
            {'x': 1.0, 'y': 0.0},
            {'x': 1.0, 'y': 1.0},
            {'x': 0.0, 'y': 1.0},
            {'x': 0.0, 'y': 0.0}
        ]
        self.current_wp = 0
        self.retry_count = 0
        
        # Setup logging
        self.log_file = os.path.join(
            os.path.expanduser('~/nam_ws/src/nam_patrol_robot/logs/'),
            f"patrol_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        self.init_log()
        
        # Signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Start patrol system
        self.get_logger().info("Initializing patrol system...")
        self.navigator.waitUntilNav2Active()
        self.create_timer(0.5, self.run_patrol)
        self._shutdown_callback = self.shutdown_callback  # Keep reference

    def init_log(self):
        """Initialize log file with headers"""
        try:
            with open(self.log_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp", "Waypoint", "X", "Y", "Status", "Retries"])
        except Exception as e:
            self.get_logger().error(f"Log initialization failed: {str(e)}")

    def create_pose(self, x, y):
        """Create a navigation goal pose"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def check_tf_transform(self):
        """Verify TF transform availability with timeout"""
        try:
            self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(seconds=0),  # Get latest available transform
                timeout=rclpy.duration.Duration(seconds=self.get_parameter('tf_timeout').value)
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(f"TF error: {str(e)}")
            return False

    def run_patrol(self):
        """Main patrol control loop"""
        if not self.check_tf_transform():
            return

        if self.navigator.isTaskComplete():
            if self.current_wp < len(self.waypoints):
                self.navigate_to_waypoint()
            else:
                self.handle_patrol_completion()

    def navigate_to_waypoint(self):
        """Execute waypoint navigation"""
        wp = self.waypoints[self.current_wp]
        goal_pose = self.create_pose(wp['x'], wp['y'])
        
        self.get_logger().info(
            f"Navigating to waypoint {self.current_wp+1}: ({wp['x']}, {wp['y']})"
        )
        
        try:
            self.navigator.goToPose(goal_pose)
            self.current_wp += 1
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {str(e)}")
            self.handle_navigation_error()

    def handle_navigation_error(self):
        """Retry or advance to next waypoint"""
        max_retries = self.get_parameter('retry_attempts').value
        if self.retry_count < max_retries:
            self.retry_count += 1
            self.get_logger().warning(f"Retry #{self.retry_count} for waypoint {self.current_wp+1}")
        else:
            self.log_waypoint('FAILED')
            self.current_wp += 1
            self.retry_count = 0

    def handle_patrol_completion(self):
        """Reset patrol cycle"""
        self.log_waypoint('COMPLETED')
        self.get_logger().info("Patrol cycle complete. Restarting...")
        self.current_wp = 0
        self.retry_count = 0

    def log_waypoint(self, status):
        """Record waypoint attempt"""
        try:
            wp = self.waypoints[self.current_wp - 1]
            with open(self.log_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(),
                    self.current_wp,
                    wp['x'],
                    wp['y'],
                    status,
                    self.retry_count
                ])
        except Exception as e:
            self.get_logger().error(f"Logging failed: {str(e)}")

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info("Received shutdown signal, terminating...")
        self.destroy_node()
        rclpy.shutdown()

    def shutdown_callback(self):
        """Cleanup resources before shutdown"""
        self.get_logger().info("Performing pre-shutdown cleanup")
        self.navigator.lifecycleShutdown()
        if hasattr(self, 'tf_listener'):
            del self.tf_listener

def main():
    rclpy.init()
    try:
        patrol_node = PatrolNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(patrol_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            patrol_node.destroy_node()
            
    except KeyboardInterrupt:
        patrol_node.get_logger().info("Patrol interrupted by user")
    except Exception as e:
        patrol_node.get_logger().fatal(f"Critical failure: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
