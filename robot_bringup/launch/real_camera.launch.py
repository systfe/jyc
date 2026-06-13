from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package_name, *parts):
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_params_file = LaunchConfiguration('camera_params_file')
    show_preview = LaunchConfiguration('show_preview')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/robot/image_raw'),
        DeclareLaunchArgument('show_preview', default_value='false'),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=package_file('robocon_localization', 'config', 'camera.yaml'),
        ),

        Node(
            package='robocon_localization',
            executable='rtsp_camera_publisher.py',
            name='rtsp_camera_publisher',
            output='screen',
            parameters=[
                camera_params_file,
                {
                    'image_topic': image_topic,
                    'show_preview': ParameterValue(show_preview, value_type=bool),
                },
            ],
        ),
    ])
