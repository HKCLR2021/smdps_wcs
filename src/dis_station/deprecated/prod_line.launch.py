import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = "dis_station"
    bringup_dir = get_package_share_directory(pkg_name)

    hkclr_ip = LaunchConfiguration("hkclr_ip")
    hkclr_port = LaunchConfiguration("hkclr_port")
    jinli_ip = LaunchConfiguration("jinli_ip")
    jinli_port = LaunchConfiguration("jinli_port")
    no_of_dis_station = LaunchConfiguration("no_of_dis_station")
    no_of_pkg_mac = LaunchConfiguration("no_of_pkg_mac")
    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    declare_hkclr_ip_cmd = DeclareLaunchArgument(
        "hkclr_ip",
        default_value="127.0.0.1",
        description="HKCLR IPC IP address",
    )

    declare_hkclr_port_cmd = DeclareLaunchArgument(
        "hkclr_port",
        default_value="8000",
        description="HKCLR IPC port number",
    )

    declare_jinli_ip_cmd = DeclareLaunchArgument(
        "jinli_ip",
        default_value="127.0.0.1",
        description="Jinli IPC IP address",
    )

    declare_jinli_port_cmd = DeclareLaunchArgument(
        "jinli_port",
        default_value="8080",
        description="Jinli IPC port number",
    )

    declare_no_of_dis_station_cmd = DeclareLaunchArgument(
        "no_of_dis_station",
        default_value="0",
        description="Number of dispenser station",
    )

    declare_no_of_pkg_mac_cmd = DeclareLaunchArgument(
        "no_of_pkg_mac",
        default_value="0",
        description="Number of packaging machine",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes.",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "dis_station.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    ld.add_action(declare_hkclr_ip_cmd)
    ld.add_action(declare_hkclr_port_cmd)
    ld.add_action(declare_jinli_ip_cmd)
    ld.add_action(declare_jinli_port_cmd)
    ld.add_action(declare_no_of_dis_station_cmd)
    ld.add_action(declare_no_of_pkg_mac_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    prod_line_ctrl = Node(
        package=pkg_name,
        executable="prod_line_ctrl",
        name="prod_line_ctrl",
        parameters=[
            params_file,
            {"no_of_dis_station": no_of_dis_station},
            {"no_of_pkg_mac": no_of_pkg_mac},
        ],
        respawn=use_respawn,
        respawn_delay=1.0,
        output="screen",
    )

    ld.add_action(prod_line_ctrl)

    return ld