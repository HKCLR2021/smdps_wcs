import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = "packaging_machine_control_system"
    bringup_dir = get_package_share_directory(pkg_name)

    no_of_pkg_mac = LaunchConfiguration("no_of_pkg_mac")
    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")
    
    declare_no_of_pkg_mac_cmd = DeclareLaunchArgument(
        "no_of_pkg_mac",
        default_value="0",
        description="Number of Packaging Machines.",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes.",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "control_system.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    ld.add_action(declare_no_of_pkg_mac_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    def prepare_multiple_nodes(context, *args, **kwargs):
        for i in range(0, int(no_of_pkg_mac.perform(context))):
            node = Node(
                package=pkg_name,
                namespace=f"packaging_machine_{i+1}",
                executable="packaging_machine_node",
                name="pkg_mac_node",
                parameters=[
                    params_file,
                    {"packaging_machine_id": i+1},
                ],
                respawn=use_respawn,
                respawn_delay=3.0,
                output="screen",
            )
            ld.add_action(node)

    ld.add_action(OpaqueFunction(function=prepare_multiple_nodes, args=[ld]))

    return ld