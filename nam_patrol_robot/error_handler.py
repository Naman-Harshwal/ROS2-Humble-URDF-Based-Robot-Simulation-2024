#!/usr/bin/env python3
import rclpy
import signal
import asyncio
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
from std_msgs.msg import String
from datetime import datetime


class NavErrorHandler(Node):
    def __init__(self):
        super().__init__('nav_error_handler')
        self.declare_parameter('retry_limit', 3)
        self.declare_parameter('retry_delay', 5.0)
        self.declare_parameter('response_timeout', 2.0)
        
        self.node_retries = {
            'controller_server': 0,
            'planner_server': 0,
            'slam_toolbox': 0
        }
        
        # Service clients for node monitoring
        self.service_clients = {
            node: self.create_client(GetState, f'/mobile/{node}/get_state')
            for node in self.node_retries.keys()
        }
        
        
        # Add transition labels dictionary
        self.transition_labels = {
            0: 'CREATE',
            1: 'CONFIGURE',
            2: 'CLEANUP',
            3: 'ACTIVATE', 
            4: 'DEACTIVATE',
            5: 'SHUTDOWN',
            6: 'UNCONFIGURED_SHUTDOWN',
            7: 'INACTIVE_SHUTDOWN'
        }
        
        
        # Setup debug publisher
        self.debug_pub = self.create_publisher(String, '/nav_error_handler/debug', 10)
        
        # Start monitoring
        self.monitor_timer = self.create_timer(5.0, self.monitor_nodes)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        
        
    def signal_handler(self, signum, frame):
        self.get_logger().info('Received shutdown signal')
        self.destroy_node()
        rclpy.shutdown()

    async def monitor_nodes(self):
        """Check node status and handle failures"""
        for node_name, client in self.service_clients.items():
            if not client.service_is_ready():
                self.get_logger().warning(f'{node_name} service not available')
                continue
                
            try:
                future = client.call_async(GetState.Request())
                await self.wait_for_future(future, node_name)
                
                if future.result() is not None:
                    state = future.result().current_state.id
                    if state != State.PRIMARY_STATE_ACTIVE:
                        self.handle_inactive_node(node_name, state)
                        
            except Exception as e:
                self.get_logger().error(f'{node_name} check failed: {str(e)}')
                self.handle_node_failure(node_name)

    async def wait_for_future(self, future, node_name):
        """Wait for service response with timeout"""
        try:
            await asyncio.wait_for(
                rclpy.spin_until_future_complete(self, future),
                timeout=self.get_parameter('response_timeout').value
            )
        except (asyncio.TimeoutError, AttributeError) as e:
            self.get_logger().warning(f'{node_name} check timeout: {str(e)}')
            raise

    def handle_inactive_node(self, node_name, state):
        """Handle non-active node states"""
        self.get_logger().warning(
            f'{node_name} in state {State.PRIMARY_STATE_LABELS[state]}'
        )
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.restart_node(node_name, Transition.TRANSITION_CONFIGURE)
        elif state == State.PRIMARY_STATE_INACTIVE:
            self.restart_node(node_name, Transition.TRANSITION_ACTIVATE)

    def handle_node_failure(self, node_name):
        """Manage node failure recovery"""
        if self.node_retries[node_name] < self.get_parameter('retry_limit').value:
            self.node_retries[node_name] += 1
            self.get_logger().info(
                f'Attempting recovery of {node_name} '
                f'(retry {self.node_retries[node_name]})'
            )
            self.restart_node(node_name, Transition.TRANSITION_ACTIVATE)
        else:
            self.get_logger().fatal(
                f'Max retries reached for {node_name}! Shutting down...'
            )
            self.emergency_shutdown()

    def restart_node(self, node_name, transition):
        """Execute lifecycle state transition"""
        client = self.create_client(ChangeState, f'/{node_name}/change_state')
        if not client.wait_for_service(
            timeout_sec=self.get_parameter('response_timeout').value
        ):
            self.get_logger().error(f'{node_name} service unavailable')
            return
            
        req = ChangeState.Request()
        req.transition.id = transition
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.transition_callback(f, node_name, transition)
        )

    def transition_callback(self, future, node_name, transition):
        try:
            response = future.result()
            if response.success:
                label = self.transition_labels.get(transition.id, 'UNKNOWN')
                self.get_logger().info(
                    f"{node_name} transition {label} successful"
                )
            else:
                label = self.transition_labels.get(transition.id, 'UNKNOWN')
                self.get_logger().error(
                    f"{node_name} transition {label} failed"
                )
        except Exception as e:
            self.get_logger().error(f"Transition callback error: {str(e)}")

    def emergency_shutdown(self):
        """Graceful system shutdown procedure"""
        self.debug_pub.publish(String(data="EMERGENCY_SHUTDOWN_INITIATED"))
        self.get_logger().fatal("Initiating emergency shutdown sequence")
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    try:
        handler = NavErrorHandler()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(handler)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            handler.destroy_node()
            
    except KeyboardInterrupt:
        handler.get_logger().info("Shutdown by user request")
    except Exception as e:
        handler.get_logger().fatal(f"Critical failure: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
