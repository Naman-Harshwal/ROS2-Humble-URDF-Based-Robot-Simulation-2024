from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('nam_patrol_robot')
    
    return LaunchDescription([
        # ==== Parameters ====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # ==== Robot State Publisher ====
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', 
                    PathJoinSubstitution([pkg_path, 'urdf', 'robot.urdf'])
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': 100.0
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        # ==== SLAM Toolbox Node ====
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='mobile',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_path, 'config', 'slam.yaml']),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'odom_frame': 'mobile/odom',
                    'map_frame': 'map',
                    'base_frame': 'mobile/base_footprint',
                    'publish_map_odom_transform': True
                }
            ],
            remappings=[
                ('/scan', '/mobile/scan'),
                ('/tf', '/mobile/tf'),
                ('/tf_static', '/mobile/tf_static')
            ]
        ),
        
        # ==== SLAM Lifecycle Manager ====
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='slam_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 15.0,  # Increased timeout
                'service_timeout': 7.0
            }]
        ),
        
        # ==== Shutdown Handlers ====
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action='slam_toolbox',
                on_exit=[
                    LogInfo(msg='SLAM exited, shutting down...'),
                    EmitEvent(event=Shutdown(reason='SLAM exited'))
                ]
        ),
        
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    LogInfo(msg='Launch system shutting down. Stopping SLAM...'),
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='slam_shutdown_manager',
                        output='screen',
                        parameters=[{
                            'node_names': ['slam_toolbox'],
                            'autostart': True
                        }],
                        on_exit=[
                            LogInfo(msg='SLAM components shut down'),
                            EmitEvent(event=Shutdown())
                        ]
                    )
                ]
            )
        )
    ])
