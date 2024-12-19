from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    NUMBER_OF_DISPENSER_STATION = 1

    for i in range(1, NUMBER_OF_DISPENSER_STATION + 1):
        node = Node(
            package="wcs",
            namespace=f"dispenser_station_{i}",
            executable="dis_station_node",
            # name=f"packaging_machine_node_{i}",
            parameters=[
                {"id": i},
                {"ip": "127.0.0.1"},
                {"port": "4840"},
            ],
            respawn=True,
            respawn_delay=3,
            output="screen",
          )
        ld.add_action(node)

    prod_line_ctrl = Node(
        package="wcs",
        executable="prod_line_ctrl",
        name="prod_line_ctrl",
        parameters=[
            {"hkclr_ip": "127.0.0.1"},
            {"hkclr_port": 8000},
            {"jinli_ip": "192.168.0.102"},
            {"jinli_port": 8080},
          ],
        respawn=True,
        respawn_delay=5,
        output="screen",
      )
    ld.add_action(prod_line_ctrl)

    return ld