from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_point = LaunchConfiguration('start_point')

    localization_launch = PathJoinSubstitution([
        FindPackageShare('robocon_localization'),
        'launch',
        'localization.launch',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('start_point', default_value='1'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'gui': 'true',
                'verbose': 'false',
                'paused': 'false',
                'start_point': start_point,
            }.items(),
        ),

        Node(
            package='robot_bringup',
            executable='interface_bridge',
            name='interface_bridge',
            output='screen',
            parameters=[{
                'cmd_input_topic': '/robot/cmd_vel',
                'cmd_output_topic': '/cmd_vel',
                'image_input_topic': '/omni_camera/image_raw',
                'image_output_topic': '/robot/image_raw',
                'odom_input_topic': '/odom',
                'odom_output_topic': '/robot/odom',
            }],
        ),
    ])
