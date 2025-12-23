from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- 1. Define Launch Arguments ---
    
    # NEW: Argument to control the delay time
    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='5.0',
        description='Seconds to wait before starting control/actuation nodes'
    )

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='true',
        description='Enable or disable video recording'
    )

    camera_prefix_arg = DeclareLaunchArgument(
        'camera_prefix',
        default_value='/camera1_HV0130315L0317',
        description='Topic prefix for the camera'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/rppi4/workspace/acds_ws/src/acds_perception/acds_perception/models/best.pt',
        description='Absolute path to the YOLO .pt file'
    )

    # --- 2. Define The Perception Node (Starts Immediately) ---
    integrated_perception_node = Node(
        package='acds_perception',
        executable='integrated_perception_node',
        name='integrated_perception_node',
        output='screen',
        parameters=[{
            'camera_prefix': LaunchConfiguration('camera_prefix'),
            'model_path': LaunchConfiguration('model_path'),
            'record': LaunchConfiguration('record'),
            'output_path': '/home/rppi4/workspace/acds_ws/integrated_output.avi'
        }]
    )

    # --- 3. Define Dependent Nodes (Control & Actuation) ---
    # These are grouped together but NOT put in the description yet.
    # We will wrap them in a timer below.
    dependent_nodes = [
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
    ]

    # --- 4. Create the Timer Action ---
    # This tells ROS: "Wait 'startup_delay' seconds, then run 'dependent_nodes'"
    delayed_start = TimerAction(
        period=LaunchConfiguration('startup_delay'),
        actions=dependent_nodes
    )

    return LaunchDescription([
        # Arguments
        startup_delay_arg,
        record_arg,
        camera_prefix_arg,
        model_path_arg,
        
        # 1. Start Perception NOW
        integrated_perception_node,
        
        # 2. Start everything else LATER (after 5 seconds)
        delayed_start
    ])