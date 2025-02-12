import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = "wcs"

    bringup_dir = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(bringup_dir, "launch")

    no_of_dis_station = LaunchConfiguration("no_of_dis_station")
    no_of_pkg_mac = LaunchConfiguration("no_of_pkg_mac")
    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    declare_no_of_dis_station_cmd = DeclareLaunchArgument(
        "no_of_dis_station",
        default_value="True",
        description="Number of Dispenser Station",
    )

    declare_no_of_pkg_mac_cmd = DeclareLaunchArgument(
        "no_of_pkg_mac",
        default_value="True",
        description="Number of Dispenser Station",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes.",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "wcs.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    ld.add_action(declare_no_of_dis_station_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "prod_line.launch.py")
            ),
            launch_arguments={
                "no_of_dis_station": no_of_dis_station,
                "no_of_pkg_mac": no_of_pkg_mac,
                "params_file": params_file,
                "use_respawn": use_respawn,
            }.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "dis_station.launch.py")
            ),
            launch_arguments={
                "no_of_dis_station": no_of_dis_station,
                "params_file": params_file,
                "use_respawn": use_respawn,
            }.items(),
        )
    )

    return ld