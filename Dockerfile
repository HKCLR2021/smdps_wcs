FROM ubuntu:22.04 AS ros2_img

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
ENV ROS_DISTRO=humble
RUN apt install -y ros-${ROS_DISTRO}-ros-base

RUN apt install -y build-essential cmake git wget dos2unix \
    python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool

FROM ros2_img:latest AS ros2_canopen_img

RUN apt install -y \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-controller-interface \ 
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-controller-manager \
    libusb-1.0-0-dev \
    autoconf automake autotools-dev gfortran gfortran-11 ibverbs-providers \
    icu-devtools libboost-all-dev libboost-atomic-dev libboost-atomic1.74-dev \
    libboost-atomic1.74.0 libboost-chrono-dev libboost-chrono1.74-dev \
    libboost-chrono1.74.0 libboost-container-dev libboost-container1.74-dev \
    libboost-container1.74.0 libboost-context-dev libboost-context1.74-dev \
    libboost-context1.74.0 libboost-coroutine-dev libboost-coroutine1.74-dev \
    libboost-coroutine1.74.0 libboost-date-time-dev libboost-date-time1.74-dev \
    libboost-date-time1.74.0 libboost-dev libboost-exception-dev \
    libboost-exception1.74-dev libboost-fiber-dev libboost-fiber1.74-dev \
    libboost-fiber1.74.0 libboost-filesystem-dev libboost-filesystem1.74-dev \
    libboost-filesystem1.74.0 libboost-graph-dev libboost-graph-parallel-dev \
    libboost-graph-parallel1.74-dev libboost-graph-parallel1.74.0 \
    libboost-graph1.74-dev libboost-graph1.74.0 libboost-iostreams-dev \
    libboost-iostreams1.74-dev libboost-iostreams1.74.0 libboost-locale-dev \
    libboost-locale1.74-dev libboost-locale1.74.0 libboost-log-dev \
    libboost-log1.74-dev libboost-log1.74.0 libboost-math-dev \
    libboost-math1.74-dev libboost-math1.74.0 libboost-mpi-dev \
    libboost-mpi-python-dev libboost-mpi-python1.74-dev \
    libboost-mpi-python1.74.0 libboost-mpi1.74-dev libboost-mpi1.74.0 \
    libboost-nowide-dev libboost-nowide1.74-dev libboost-nowide1.74.0 \
    libboost-numpy-dev libboost-numpy1.74-dev libboost-numpy1.74.0 \
    libboost-program-options-dev libboost-program-options1.74-dev \
    libboost-program-options1.74.0 libboost-python-dev libboost-python1.74-dev \
    libboost-python1.74.0 libboost-random-dev libboost-random1.74-dev \
    libboost-random1.74.0 libboost-regex-dev libboost-regex1.74-dev \
    libboost-regex1.74.0 libboost-serialization-dev \
    libboost-serialization1.74-dev libboost-serialization1.74.0 \
    libboost-stacktrace-dev libboost-stacktrace1.74-dev \
    libboost-stacktrace1.74.0 libboost-system-dev libboost-system1.74-dev \
    libboost-system1.74.0 libboost-test-dev libboost-test1.74-dev \
    libboost-test1.74.0 libboost-thread-dev libboost-thread1.74-dev \
    libboost-thread1.74.0 libboost-timer-dev libboost-timer1.74-dev \
    libboost-timer1.74.0 libboost-tools-dev libboost-type-erasure-dev \
    libboost-type-erasure1.74-dev libboost-type-erasure1.74.0 libboost-wave-dev \
    libboost-wave1.74-dev libboost-wave1.74.0 libboost1.74-dev \
    libboost1.74-tools-dev libcaf-openmpi-3 libcoarrays-dev \
    libcoarrays-openmpi-dev libevent-2.1-7 libevent-core-2.1-7 libevent-dev \
    libevent-extra-2.1-7 libevent-openssl-2.1-7 libevent-pthreads-2.1-7 \
    libfabric1 libgfortran-11-dev libhwloc-dev libhwloc-plugins libhwloc15 \
    libibverbs-dev libibverbs1 libicu-dev libjs-jquery-ui libltdl-dev \
    libnl-3-200 libnl-3-dev libnl-route-3-200 libnl-route-3-dev libnuma-dev \
    libnuma1 libopenmpi-dev libopenmpi3 libpciaccess0 libpmix-dev libpmix2 \
    libpsm-infinipath1 libpsm2-2 librdmacm1 libsigsegv2 libtool libucx0 \
    libxnvctrl0 m4 mpi-default-bin mpi-default-dev ocl-icd-libopencl1 \
    openmpi-bin openmpi-common libtool-bin
    
FROM ros2_canopen_img:latest AS pkg_mac_sys

EXPOSE 9090 11811

ENV WS_NAME=packaging_system
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src/packaging_machine_comm ./src/packaging_machine_comm
COPY ./src/packaging_machine_control_system ./src/packaging_machine_control_system

COPY ./src/smdps_msgs ./src/smdps_msgs

COPY ./src/ros2_canopen ./src/ros2_canopen

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs", "/dev/bus", "/dev/serial" ]

COPY --chmod=755 ./docker/entrypoint.sh /
RUN dos2unix /entrypoint.sh 
ENTRYPOINT [ "/entrypoint.sh" ]

FROM ros2_img:latest AS prod_line_sys

EXPOSE 8000

ENV WS_NAME=smdps_wcs
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src/wcs ./src/wcs

COPY ./src/smdps_msgs ./src/smdps_msgs

COPY ./src/nlohmann ./src/nlohmann
COPY ./src/open62541pp ./src/open62541pp

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs" ]

COPY --chmod=755 ./docker/entrypoint.sh /
RUN dos2unix /entrypoint.sh 
ENTRYPOINT [ "/entrypoint.sh" ]