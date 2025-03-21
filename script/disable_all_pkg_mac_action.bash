#! /bin/bash

CONTAINER_NAME=pkg_mac
WS_NAME=smdps_wcs
ROS_DISTRO=humble

docker exec $CONTAINER_NAME /bin/bash -c "\
    source /opt/ros/$ROS_DISTRO/setup.bash; \
    source /$WS_NAME/install/setup.bash; \
    ros2 service call /packaging_machine_1/heater_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    ros2 service call /packaging_machine_2/heater_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    ros2 service call /packaging_machine_1/conveyor_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    ros2 service call /packaging_machine_2/conveyor_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    ros2 service call /packaging_machine_1/stopper_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    ros2 service call /packaging_machine_2/stopper_operation std_srvs/srv/SetBool \"{data: 0}\"; \
    "