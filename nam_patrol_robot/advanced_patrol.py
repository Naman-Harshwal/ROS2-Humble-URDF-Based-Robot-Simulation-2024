#!/usr/bin/env python3
import rclpy
import csv
import os
import signal
import math
from datetime import datetime
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State

class AdvancedPatrol(Node):
    def __init__(self):
        super().__init__('advanced_patrol')
        self.navigator = BasicNavigator()
        self.waypoints = self.create_waypoints()
        self.current_goal = 0
        self.obstacle_detected = False
        self.retry_count = 0
        self.active = True
        
        # Signal handling for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('retry_attempts', 3),
                ('obstacle_threshold', 0.3),
                ('waypoint_hold_time', 5.0),
                ('replanning_delay', 5.0)
            ]
        )
        
        # Costmap subscription
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )
        
        # Initialize logging
        self.log_file = os.path.join(
            os.path.expanduser('~/nam_ws/src/nam_patrol_robot/logs/'),
            f"patrol_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        self.init_logging()
        
        # AMCL status monitoring
        self.amcl_ready = False
        self.check_amcl_timer = self.create_timer(1.0, self.check_amcl_status)
        
        # Navigation timer
        self.patrol_timer = None

    def signal_handler(self, signum, frame):
        self.get_logger().info('Received SIGINT, shutting down...')
        self.destroy_node()
        rclpy.try_shutdown()

    def _shutdown_callback(self):
        """Cleanup resources before shutdown"""
        self.get_logger().info('Performing pre-shutdown cleanup')
        # Cancel timers
        if hasattr(self, 'patrol_timer'):
            self.destroy_timer(self.patrol_timer)
        # Destroy subscriptions/publishers
        self.destroy_subscription(self.costmap_sub)

    def check_amcl_status(self):
        try:
            client = self.create_client(GetState, '/amcl/get_state')
            if client.wait_for_service(timeout_sec=1.0):
                req = GetState.Request()
                future = client.call_async(req)
                future.add_done_callback(self._amcl_status_callback)
        except Exception as e:
            self.get_logger().warning(f"AMCL status check failed: {str(e)}")

    def _amcl_status_callback(self, future):
        try:
            response = future.result()
            if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                if not self.amcl_ready:
                    self.get_logger().info("AMCL activated! Starting patrol...")
                    self.amcl_ready = True
                    self.destroy_timer(self.check_amcl_timer)
                    self.start_patrol()
        except Exception as e:
            self.get_logger().error(f"AMCL callback error: {str(e)}")

    def start_patrol(self):
        """Initialize patrol system"""
        try:
            self.navigator.waitUntilNav2Active(navigator="bt_navigator")
            self.patrol_timer = self.create_timer(
                0.1,  # 10 Hz control loop
                self.patrol_control_loop
            )
        except Exception as e:
            self.get_logger().fatal(f"Failed to start patrol: {str(e)}")
            raise

    def create_waypoints(self):
        """Generate navigation waypoints"""
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        return [
            self.create_pose(header, 2.0, 0.0),
            self.create_pose(header, 4.0, 2.0),
            self.create_pose(header, 2.0, 4.0),
            self.create_pose(header, 0.0, 4.0),
            self.create_pose(header, 0.0, 2.0),
            self.create_pose(header, 2.0, 2.0)
        ]

    def create_pose(self, header, x, y):
        """Create PoseStamped message"""
        pose = PoseStamped(
            header=header,
            pose=Pose(
                position=Point(x=float(x), y=float(y), z=0.0),
                orientation=Quaternion(w=1.0)
        ))
        return pose

    def costmap_callback(self, msg):
        """Process costmap data for obstacle detection"""
        try:
            if not self.amcl_ready or not self.active:
                return

            occupied = sum(1 for cell in msg.data if cell > 65)
            total = len(msg.data)
            threshold = self.get_parameter('obstacle_threshold').value
            self.obstacle_detected = (occupied / total) > threshold

            if self.obstacle_detected:
                self.get_logger().warning("Obstacle detected! Initiating avoidance...")
                self.handle_obstacle()
                
        except Exception as e:
            self.get_logger().error(f"Costmap error: {str(e)}")

    def patrol_control_loop(self):
        """Main navigation control loop"""
        if not self.active or self.current_goal >= len(self.waypoints):
            self._complete_patrol()
            return

        try:
            if not self.navigator.isTaskComplete():
                return

            goal = self.waypoints[self.current_goal]
            self.navigator.goToPose(goal)
            self.get_logger().info(
                f"Navigating to waypoint {self.current_goal+1}: "
                f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
            )

        except Exception as e:
            self.get_logger().error(f"Navigation error: {str(e)}")
            self.retry_count += 1
            self._handle_retries()

    def _complete_patrol(self):
        """Handle patrol completion"""
        self.get_logger().info("Patrol cycle completed successfully!")
        self.patrol_timer.cancel()
        self.active = False

    def _handle_retries(self):
        """Manage navigation retry logic"""
        max_retries = self.get_parameter('retry_attempts').value
        if self.retry_count >= max_retries:
            self.log_waypoint('FAILED', self.retry_count)
            self.current_goal += 1
            self.retry_count = 0
            self.get_logger().error(f"Aborting waypoint {self.current_goal}")
        else:
            self.get_logger().info(f"Retrying waypoint {self.current_goal+1} (attempt {self.retry_count+1}/{max_retries})")
            self.create_timer(2.0, self.patrol_control_loop)

    def handle_obstacle(self):
        """Obstacle avoidance strategy"""
        self.navigator.cancelTask()
        self.current_goal = (self.current_goal + 1) % len(self.waypoints)
        self.obstacle_detected = False
        self.retry_count = 0
        self.get_logger().info(f"Replanning to waypoint {self.current_goal+1}")
        self.create_timer(
            self.get_parameter('replanning_delay').value,
            self.patrol_control_loop
        )

    def init_logging(self):
        """Initialize CSV log file"""
        with open(self.log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Timestamp', 'Waypoint', 'X', 'Y',
                'Status', 'Retries', 'Obstacles'
            ])

    def log_waypoint(self, status, retries):
        """Record patrol progress"""
        wp = self.waypoints[self.current_goal].pose.position
        try:
            with open(self.log_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(),
                    self.current_goal + 1,
                    round(wp.x, 2),
                    round(wp.y, 2),
                    status,
                    retries,
                    self.obstacle_detected
                ])
        except Exception as e:
            self.get_logger().error(f"Logging failed: {str(e)}")

def main():
    rclpy.init()
    patrol_node = AdvancedPatrol()
    try:
        patrol_node = AdvancedPatrol()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(patrol_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            patrol_node.destroy_node()

    except KeyboardInterrupt:
        patrol_node.get_logger().info("Patrol interrupted by user")
    except Exception as e:
        if patrol_node:  # Check if node was created
            patrol_node.get_logger().fatal(f"Critical failure: {str(e)}")
        else:
            print(f"Critical failure before node creation: {str(e)}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
