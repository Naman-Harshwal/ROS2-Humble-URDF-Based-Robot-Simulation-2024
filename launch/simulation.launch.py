import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown  # CRITICAL MISSING IMPORT
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('nam_patrol_robot')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    

    return LaunchDescription([
        # 1. Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'), 
                    'launch', 
                    'gazebo.launch.py'
                )
            ]),
            launch_arguments={
                'world': os.path.join(pkg_path, 'worlds', 'my_customised_world.world'),
                'verbose': 'false',
                'pause': 'false'
            }.items()
        ),

        # 2. Robot State Publisher with TF remapping
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(os.path.join(pkg_path, 'urdf/robot.urdf')).read(),
                'use_sim_time': True,
                'frame_prefix': 'mobile/'
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            output='screen'
        ),

        # 3. Spawn Robot Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-entity', 'patrol_robot',
                '-topic', 'robot_description',
                '-z', '0.1',
                '-x', '0.0',
                '-y', '0.0',
                '-Y', '0.0',
                '-timeout', '60'
            ],
            output='screen'
        ),

        # 4. SLAM Toolbox Configuration
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[
                os.path.join(pkg_path, 'config/slam.yaml'),
                {
                    'use_sim_time': True,
                    'odom_frame': 'mobile/odom',
                    'base_frame': 'mobile/base_footprint',
                    'map_frame': 'map'
                }
            ],
            remappings=[
                ('/scan', '/mobile/scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            output='screen'
        ),

        # 5. SLAM Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='slam_lifecycle_manager',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox']
            }],
            output='screen'
        ),

        # 6. Navigation System
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_path, 'launch/navigation_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(pkg_path, 'config/nav2_params.yaml'),
                'use_sim_time': 'True',
                'namespace': 'mobile'
            }.items()
        ),

        # 7. ROS2 Control System
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='mobile',
            parameters=[os.path.join(pkg_path, 'config/controllers.yaml')],
            output='screen'
        ),

        # 8. Controller Spawners
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py'
                ),
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package='controller_manager',
                                executable='spawner',
                                namespace='mobile',
                                arguments=['joint_state_broadcaster'],
                                output='screen'
                            ),
                            Node(
                                package='controller_manager',
                                executable='spawner',
                                namespace='mobile',
                                arguments=['mecanum_controller'],
                                output='screen'
                            )
                        ]
                    )
                ]
            )
        ),

        # 9. RViz2 with Proper Configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz/nav.rviz')],
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/initialpose', '/mobile/initialpose'),
                ('/goal_pose', '/mobile/goal_pose')
            ],
            output='screen'
        ),

        # 10. Patrol System Components
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='nam_patrol_robot',
                    executable='patrol_node.py',
                    namespace='mobile',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
                Node(
                    package='nam_patrol_robot',
                    executable='error_handler.py',
                    namespace='mobile',
                    output='screen'
                ),
                Node(
                    package='nam_patrol_robot',
                    executable='advanced_patrol.py',
                    namespace='mobile',
                    parameters=[{
                        'retry_attempts': 3,
                        'obstacle_threshold': 0.3
                    }],
                    output='screen'
                )
            ]
        ),

        # 11. System Shutdown Handlers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package='gazebo_ros',
                    executable='gzserver'
                ),
                on_exit=[
                    RegisterEventHandler(
                        event_handler=OnProcessExit(
                            target_action=Node(
                                package='rviz2',
                                executable='rviz2'
                            ),
                            on_exit=[EmitEvent(event=Shutdown())]
                        )
                    )
                ]
            )
        )
    ])
