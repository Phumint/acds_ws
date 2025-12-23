import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # --- Paths ---
    pkg_acds_launch = get_package_share_directory('acds_launch')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # FIX 1: Updated to the correct filename 'robot.urdf.xacro'
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare('acds_description'), 'urdf', 'robot.urdf.xacro'
    ])
    
    nav2_params_file = os.path.join(pkg_acds_launch, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_acds_launch, 'maps', 'map.yaml')

    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # --- 1. Robot State Publisher ---
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

    # --- 2. Hardware Nodes (Sensors/Motors) ---
    # FIX 2: Pointing to 'acds_actuation' package and correct executable names
    vehicle_driver_node = Node(
        package='acds_actuation',          # <--- Changed from acds_launch
        executable='vehicle_driver_node',  # <--- Changed to match setup.py
        name='vehicle_driver_node'
    )
    
    imu_node = Node(
        package='acds_actuation',          # <--- Changed from acds_launch
        executable='imu_node',             # <--- Changed to match setup.py
        name='imu_node'
    )

    # EKF assumes you have /odom (from driver) and /imu/data
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_acds_launch, 'config', 'ekf.yaml')],
        remappings=[("odometry/filtered", "odom")]
    )

    # --- 3. FAKE LOCALIZATION (Replaces AMCL) ---
    fake_localization_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # --- 4. Nav2 Bringup ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    # --- 5. Map Server ---
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

    # --- 6. RViz (Optional) ---
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_acds_launch, 'config', 'nav.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        
        robot_state_publisher_node,
        vehicle_driver_node,
        imu_node,
        ekf_node,
        fake_localization_node, 
        map_server_node,
        lifecycle_manager_map,
        
        TimerAction(period=3.0, actions=[nav2_launch]),
        rviz_node,
    ])