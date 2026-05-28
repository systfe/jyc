from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    paused = LaunchConfiguration('paused')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[FindPackageShare('robocon25_sim'), '/worlds/robocon25.world'],
            description='Gazebo world file',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_DATABASE_URI',
            '',
        ),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                FindPackageShare('robocon25_sim'),
                '/models:',
                FindPackageShare('robocon25_sim'),
                '/..:',
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('gazebo_ros'),
                '/launch/gzserver.launch.py',
            ]),
            launch_arguments={
                'world': world,
                'verbose': verbose,
                'pause': paused,
            }.items(),
        ),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(gui),
        ),
    ])
