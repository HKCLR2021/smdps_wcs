# ========== ros2_img ========== 
FROM ubuntu:22.04 AS ros2_img

ARG http_proxy
ARG https_proxy
ARG all_proxy

RUN echo $http_proxy
RUN echo $https_proxy
RUN echo $all_proxy

RUN apt update
RUN DEBIAN_FRONTEND="noninteractive" apt install -y tzdata

ENV TZ=Asia/Hong_Kong
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata 

RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt install -y curl systemd udev software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \ 
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt -y upgrade

ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0

RUN apt install -y ros-${ROS_DISTRO}-ros-base

RUN apt install -y build-essential cmake git wget dos2unix \
    python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool

COPY --chmod=755 ./docker/entrypoint.sh /
RUN dos2unix /entrypoint.sh 
ENTRYPOINT [ "/entrypoint.sh" ]

# ========== ros2_canopen_img ========== 
FROM ros2_img:latest AS ros2_canopen_img

RUN apt install -y ros-${ROS_DISTRO}-rosbridge-server libusb-1.0-0-dev 

ENV WS_NAME=smdps_wcs
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src/smdps_msgs ./src/smdps_msgs
COPY ./src/ros2_canopen ./src/ros2_canopen

RUN rosdep init
RUN rosdep fix-permissions
RUN rosdep update
RUN rosdep install --from-paths src/ros2_canopen --ignore-src -r -y

RUN mkdir /memory
VOLUME [ "/memory" ]

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN rm -rf ./src/smdps_msgs ./src/ros2_canopen

# ========== dis_station_img ========== 
FROM ros2_img:latest AS dis_station_img

ENV WS_NAME=smdps_wcs
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src/smdps_msgs ./src/smdps_msgs
COPY ./src/open62541pp ./src/open62541pp

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN rm -rf ./src/smdps_msgs ./src/open62541pp

# ========== prod_line_img ========== 
FROM ros2_img:latest AS prod_line_img

ENV WS_NAME=smdps_wcs
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src/smdps_msgs ./src/smdps_msgs
COPY ./src/qr_camera ./src/qr_camera
COPY ./src/prod_line_plc_comm ./src/prod_line_plc_comm
COPY ./src/prod_line_sys ./src/prod_line_sys

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN pip install pymodbus==3.8.6

RUN rm -rf ./src/smdps_msgs ./src/qr_camera ./src/prod_line_plc_comm ./src/prod_line_sys

# ========== pkg_mac_sys ========== 
FROM ros2_canopen_img AS pkg_mac_sys

EXPOSE 9090

COPY ./src/packaging_machine_comm ./src/packaging_machine_comm
COPY ./src/packaging_machine_control_system ./src/packaging_machine_control_system
COPY ./fastdds_profiles.xml ./

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/${WS_NAME}/fastdds_profiles.xml

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . install/setup.sh && \
    colcon build \
    --packages-select packaging_machine_comm packaging_machine_control_system \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN rm -rf ./src/packaging_machine_comm ./src/packaging_machine_control_system

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs", "/dev/bus", "/dev/serial" ]

# ========== dis_station ========== 
FROM dis_station_img AS dis_station

COPY ./src/dis_station ./src/dis_station
COPY ./fastdds_profiles.xml ./

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/${WS_NAME}/fastdds_profiles.xml

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . install/setup.sh && \
    colcon build \
    --packages-select dis_station \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN rm -rf ./src/dis_station

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs" ]

# ========== prod_line_sys ========== 
FROM prod_line_img AS prod_line_sys

COPY ./fastdds_profiles.xml ./

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/${WS_NAME}/fastdds_profiles.xml

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs" ]

