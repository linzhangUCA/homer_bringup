from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from math import pi


def generate_launch_description():
    homer_package_path = get_package_share_path("homer_bringup")
    joy_config_path = homer_package_path / "configs/ps3.config.yaml"

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )

    footprint_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "-0.0325",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_footprint",
        ],
    )

    lidar_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.021",
            "--y",
            "0",
            "--z",
            "0.12",
            "--yaw",
            str(pi),
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_link",
        ],
    )

    # diff_drive_node = Node(package="solid_octo", executable="diff_drive_controller")
    homer_interface_node = Node(package="homer_bringup", executable="homer_interface")

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(homer_package_path / "launch/rplidar.launch.py")
        ),
    )

    launch_teleop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("teleop_twist_joy") / "launch/teleop-launch.py")
        ),
        launch_arguments={
            "joy_config": "ps3",
            "config_filepath": str(joy_config_path),
        }.items(),
    )

    return LaunchDescription(
        [
            sim_time_arg,
            homer_interface_node,
            rplidar_launch,
            launch_teleop_twist_joy,
            footprint_static_tf_node,
            lidar_static_tf_node,
        ]
    )
