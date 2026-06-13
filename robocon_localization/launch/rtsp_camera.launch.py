from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package_name, *parts):
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=package_file('robocon_localization', 'config', 'rtsp_camera.yaml'),
        ),
        DeclareLaunchArgument('image_topic', default_value='/robot/image_raw'),
        Node(
            package='robocon_localization',
            executable='rtsp_camera_publisher.py',
            name='rtsp_camera_publisher',
            output='screen',
            parameters=[
                params_file,
                {'image_topic': image_topic},
            ],
        ),
    ])
