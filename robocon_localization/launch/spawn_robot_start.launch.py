from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def spawn_robot(context):
    start_point = LaunchConfiguration("start_point").perform(context)
    half_field = 1.5
    start_half = 0.15
    start_center = half_field - start_half

    # Robot is placed square to the field. For each corner there are two inward
    # axis-aligned directions; this chooses the one not facing the safety area.
    start_poses = {
        "1": (start_center, start_center, -1.570796),
        "2": (start_center, -start_center, 1.570796),
        "3": (-start_center, -start_center, 1.570796),
        "4": (-start_center, start_center, -1.570796),
    }
    x, y, yaw = start_poses.get(start_point, start_poses["1"])

    robot_model = os.path.join(
        get_package_share_directory("robocon25_sim"),
        "models",
        "robot.model",
    )

    return [
        Node(
            name="spawn_urdf",
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file", robot_model,
                "-entity", "robot",
                "-x", f"{x:.3f}",
                "-y", f"{y:.3f}",
                "-Y", f"{yaw:.6f}",
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_point",
            default_value="1",
            choices=["1", "2", "3", "4"],
            description="Robot start area number. Centers are 150mm from field edges.",
        ),
        OpaqueFunction(function=spawn_robot),
    ])
