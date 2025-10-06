from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Enable or disable video recording'
    )

    return LaunchDescription([
        record_arg,
        Node(
            package='acds_perception',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[{'record': LaunchConfiguration('record')}]
        ),
        Node(
            package='acds_control',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='acds_actuation',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),
        Node(
            package='acds_actuation',
            executable='steering_driver_node',
            name='steering_driver_node',
            output='screen'
        ),
    ])
