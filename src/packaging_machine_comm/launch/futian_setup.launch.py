#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    can_channel = LaunchConfiguration("can_channel")

    declare_can_channel_cmd = DeclareLaunchArgument(
        "can_channel",
        default_value="can0",
        description="CAN Channel",
    )
    
    ld.add_action(declare_can_channel_cmd)

    master_bin_path = os.path.join(
        get_package_share_directory("packaging_machine_comm"),
        "config",
        "futian",
        "master.bin",
    )

    if not os.path.exists(master_bin_path):
        master_bin_path = ""

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("packaging_machine_comm"),
                "config",
                "futian",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("packaging_machine_comm"),
                "config",
                "futian",
                "bus.yml",
            ),
            "can_interface_name": can_channel,
        }.items(),
    )

    ld.add_action(device_container)
    
    return ld