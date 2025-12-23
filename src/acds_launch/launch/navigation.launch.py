import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # --- Paths ---
    pkg_acds_launch = get_package_share_directory('acds_launch')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare('acds_description'), 'urdf', 'robot.urdf.xacro'
    ])
    
    nav2_params_file = os.path.join(pkg_acds_launch, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_acds_launch, 'maps', 'map.yaml')

    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # --- 1. Robot State Publisher (Starts Immediately) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file_path])
        }]
    )

    # --- 2. Hardware Nodes (Start Immediately) ---
    # IMU needs to start ASAP to begin 10s calibration
    vehicle_driver_node = Node(
        package='acds_actuation',
        executable='vehicle_driver_node',
        name='vehicle_driver_node'
    )
    
    imu_node = Node(
        package='acds_actuation',
        executable='imu_node',
        name='imu_node',
        output='screen' # Important to see "Calibration Done" message
    )

    # --- 3. EKF (Delayed) ---
    # We delay EKF so it doesn't fuse garbage data while IMU calibrates
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_acds_launch, 'config', 'ekf.yaml')],
        remappings=[("odometry/filtered", "odom")]
    )

    # --- 4. Localization & Map (Delayed) ---
    fake_localization_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['4.57', '2.7', '0.0', '1.57', '0', '0', 'map', 'odom']
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file}]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # --- 5. Nav2 Bringup (Delayed) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_velocity_smoother': 'False', # Disable the smoother
        }.items()
    )

    # --- 6. RViz ---
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_acds_launch, 'config', 'nav.rviz')],
    )

    # --- DELAY MECHANISM ---
    # Group everything that depends on good sensor data
    delayed_launch_group = TimerAction(
        period=12.0, # 10s calibration + 2s buffer
        actions=[
            ekf_node,
            fake_localization_node,
            map_server_node,
            lifecycle_manager_map,
            nav2_launch,
            rviz_node
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        
        # 1. Start Hardware & TF Tree immediately
        robot_state_publisher_node,
        vehicle_driver_node,
        imu_node,
        
        # 2. Start Intelligence after 12 seconds
        delayed_launch_group
    ])