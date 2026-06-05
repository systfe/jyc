from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package_name, *parts):
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def spawn_robot(context):
    start_point = LaunchConfiguration('start_point').perform(context)
    half_field = 1.5
    start_half = 0.15
    start_center = half_field - start_half

    start_poses = {
        '1': (start_center, start_center, -1.570796),
        '2': (start_center, -start_center, 1.570796),
        '3': (-start_center, -start_center, 1.570796),
        '4': (-start_center, start_center, -1.570796),
    }
    x, y, yaw = start_poses.get(start_point, start_poses['1'])

    return [
        Node(
            name='spawn_urdf',
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', package_file('robocon25_sim', 'models', 'robot.model'),
                '-entity', 'robot',
                '-x', f'{x:.3f}',
                '-y', f'{y:.3f}',
                '-Y', f'{yaw:.6f}',
            ],
        )
    ]


def generate_launch_description():
    gazebo_launch = package_file('robocon25_sim', 'launch', 'gazebo_no_eol.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_point',
            default_value='1',
            choices=['1', '2', '3', '4'],
            description='Robot start area number.',
        ),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': package_file('robocon25_sim', 'worlds', 'robocon25.world'),
                'gui': LaunchConfiguration('gui'),
                'verbose': LaunchConfiguration('verbose'),
                'paused': LaunchConfiguration('paused'),
            }.items(),
        ),

        OpaqueFunction(function=spawn_robot),

        Node(
            package='robocon_localization',
            executable='omni_mirror_sim',
            name='omni_mirror_sim',
            output='screen',
            parameters=[{
                'input_topic': '/sim_camera/image_raw',
                'output_topic': '/robot/image_raw',
                'output_width': 640,
                'output_height': 480,
                'inner_radius_ratio': 0.00,
                'outer_radius_ratio': 0.98,
                'mirror_curve': 2.00,
                'source_radius_ratio': 0.98,
                'radial_k1': 0.00,
                'radial_k2': 0.00,
                'center_x_ratio': 0.50,
                'center_y_ratio': 0.50,
                'flip_radial': False,
            }],
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
                'use_odom_for_tracking': False,
                'use_safety_axis_constraint': False,
                'image_topic': '/robot/image_raw',
                'odom_topic': '/robot/odom',
                'pose_topic': '/robot/pose',
                'pose_frame_id': 'map',
            }],
        ),
    ])
