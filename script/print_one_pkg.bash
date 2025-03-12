#! /bin/bash

NO="$1"
CONTAINER_NAME=pkg_mac
WS_NAME=smdps_wcs
ROS_DISTRO=humble

docker exec $CONTAINER_NAME /bin/bash -c "\
    source /opt/ros/$ROS_DISTRO/setup.bash; \
    source /$WS_NAME/install/setup.bash; \
    ros2 service call /packaging_machine_$NO/print_one_package std_srvs/srv/Trigger \"{}\""