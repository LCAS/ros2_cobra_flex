"""Launch file for Cobra Flex driver"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Cobra Flex driver"""
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='1.0',
        description='Timeout for cmd_vel messages (seconds)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Publishing rate in Hz'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Cobra Flex driver node
    cobra_flex_node = Node(
        package='cobra_flex_driver',
        executable='cobra_flex_node',
        name='cobra_flex_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg=['Launching Cobra Flex driver on port: ', LaunchConfiguration('serial_port')]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        odom_frame_arg,
        base_frame_arg,
        publish_tf_arg,
        cmd_vel_timeout_arg,
        publish_rate_arg,
        use_sim_time_arg,
        startup_msg,
        cobra_flex_node,
    ])
