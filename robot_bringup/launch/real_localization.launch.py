from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package_name, *parts):
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_odom_for_tracking = LaunchConfiguration('use_odom_for_tracking')
    start_rtsp_camera = LaunchConfiguration('start_rtsp_camera')
    rtsp_params_file = LaunchConfiguration('rtsp_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/robot/image_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/robot/odom'),
        DeclareLaunchArgument('use_odom_for_tracking', default_value='false'),
        DeclareLaunchArgument('start_rtsp_camera', default_value='true'),
        DeclareLaunchArgument(
            'rtsp_params_file',
            default_value=package_file('robocon_localization', 'config', 'rtsp_camera.yaml'),
        ),

        Node(
            package='robocon_localization',
            executable='rtsp_camera_publisher.py',
            name='rtsp_camera_publisher',
            output='screen',
            condition=IfCondition(start_rtsp_camera),
            parameters=[
                rtsp_params_file,
                {'image_topic': image_topic},
            ],
        ),

        Node(
            package='robocon_localization',
            executable='loc_sidelines',
            name='loc_sidelines',
            output='screen',
            parameters=[{
                'table_file': package_file('robocon_localization', 'config', 'dist_table.txt'),
                'lines_file': package_file('robocon_localization', 'config', 'field_lines.png'),
                'field_file': package_file('robocon_localization', 'config', 'field_bg.png'),
                'lines_map_file': package_file('robocon_localization', 'config', 'white_lines.png'),
                'red_map_file': package_file('robocon_localization', 'config', 'red_lines.png'),
                'blue_map_file': package_file('robocon_localization', 'config', 'blue_lines.png'),
                'image_topic': image_topic,
                'odom_topic': odom_topic,
                'pose_topic': '/robot/pose',
                'pose_frame_id': 'map',
                'use_odom_for_tracking': use_odom_for_tracking,
                'use_safety_axis_constraint': False,
            }],
        ),
    ])
