#!/usr/bin/env python3
"""
Complete Navigation Launch File for ACDS Robot
Launches all components needed for Nav2 operation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    acds_launch_dir = get_package_share_directory('acds_launch')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(acds_launch_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(acds_launch_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # ========== HARDWARE DRIVERS ==========
    
    # Vehicle Driver (Motor + Servo + Odometry)
    vehicle_driver_node = Node(
        package='acds_actuation',
        executable='vehicle_driver_node',
        name='vehicle_driver_node',
        output='screen',
        parameters=[{
            'wheel_base': 0.26,
            'wheel_radius': 0.035,
            'max_steering_angle': 20.0,
            'max_speed': 1.0,
            'encoder_ticks_per_rev': 20,
            'speed_scaling': 0.3
        }]
    )
    
    # IMU Driver
    imu_node = Node(
        package='acds_actuation',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'i2c_bus': 1,
            'publish_rate': 50.0,
            'frame_id': 'imu_link'
        }]
    )
    
    # ========== ROBOT DESCRIPTION & TF ==========
    
    # Robot State Publisher (publishes URDF to /robot_description and TF)
    urdf_file = os.path.join(
        get_package_share_directory('acds_description'),
        'urdf',
        'robot.urdf.xacro'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file).read()
        }]
    )
    
    # ========== SENSOR FUSION ==========
    
    # EKF Localization (fuses /odom_raw + /imu/data -> /odometry/filtered)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(acds_launch_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('/set_pose', '/initialpose')
        ]
    )
    
    # ========== MAP SERVER ==========
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )
    
    # ========== AMCL LOCALIZATION ==========
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'scan_topic': 'scan',
                'set_initial_pose': False,
                'initial_pose': {
                    'x': 0.0,
                    'y': 0.0,
                    'yaw': 0.0
                }
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # ========== NAV2 STACK ==========
    
    # Include Nav2 bringup (launches all nav2 nodes)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )
    
    # ========== RVIZ ==========
    
    rviz_config_file = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ========== LAUNCH DESCRIPTION ==========
    
    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_rviz_cmd,
        
        # Hardware drivers
        vehicle_driver_node,
        imu_node,
        
        # Robot description & TF
        robot_state_publisher_node,
        
        # Sensor fusion
        ekf_node,
        
        # Map server
        map_server_node,
        lifecycle_manager_map,
        
        # Localization
        amcl_node,
        
        # Navigation stack
        nav2_bringup,
        
        # Visualization
        rviz_node
    ])