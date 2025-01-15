from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    NUMBER_OF_DISPENSER_STATION = 3
    NUMBER_OF_PACKAGING_MACHINE = 1

    HKCLR_IP = "172.17.0.2"
    HKCLR_PORT = 8000
    JINLI_IP = "192.168.8.211"
    JINLI_PORT = 8080

    IP_PREFIX = "192.168.8."
    OPCUA_PORT = "4840"

    for i in range(0, NUMBER_OF_DISPENSER_STATION):
        node = Node(
            package="wcs",
            namespace=f"dispenser_station_{i + 1}",
            executable="dis_station_node",
            # name=f"packaging_machine_node_{i}",
            parameters=[
                {"id": i + 1},
                {"ip": IP_PREFIX + str(100 + i + 1)}, 
                {"port": OPCUA_PORT},
            ],
            respawn=True,
            respawn_delay=1,
            output="screen",
          )
        ld.add_action(node)

    prod_line_ctrl = Node(
        package="wcs",
        executable="prod_line_ctrl",
        # name="prod_line_ctrl",
        parameters=[
            {"hkclr_ip": HKCLR_IP},
            {"hkclr_port": HKCLR_PORT},
            {"jinli_ip": JINLI_IP},
            {"jinli_port": JINLI_PORT},
            {"no_of_dispenser_stations": NUMBER_OF_DISPENSER_STATION},
            {"no_of_packaging_machine": NUMBER_OF_PACKAGING_MACHINE},
          ],
        respawn=True,
        respawn_delay=1,
        output="screen",
      )
    ld.add_action(prod_line_ctrl)

    return ld