ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
USER root
WORKDIR /workspaces

RUN rm -f /etc/apt/sources.list.d/yarn.list || true

RUN if [ "${ROS_DISTRO}" = "jazzy" ]; then \
      sed -i -E 's|http://(archive\|security)\.ubuntu\.com/ubuntu/|https://\1.ubuntu.com/ubuntu/|g' \
        /etc/apt/sources.list.d/ubuntu.sources; \
    elif [ "${ROS_DISTRO}" = "humble" ]; then \
      sed -i -E 's|http://(archive\|security)\.ubuntu\.com/ubuntu/|https://\1.ubuntu.com/ubuntu/|g' \
        /etc/apt/sources.list; \
    fi

RUN apt-get update && \
    apt-get install -y \
      ros-${ROS_DISTRO}-graph-msgs \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-diff-drive-controller \
      ros-${ROS_DISTRO}-joint-trajectory-controller \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-topic-tools \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-tf-transformations \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-ros-testing \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rqt* \
      python3-colcon-common-extensions \
      python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
      ros-${ROS_DISTRO}-moveit-ros \
      ros-${ROS_DISTRO}-moveit-servo \
      ros-${ROS_DISTRO}-moveit-planners \
      ros-${ROS_DISTRO}-moveit-plugins \
      ros-${ROS_DISTRO}-moveit-setup-assistant \
      ros-${ROS_DISTRO}-moveit-configs-utils \
      ros-${ROS_DISTRO}-moveit-task-constructor-core \
    && rm -rf /var/lib/apt/lists/*

ARG HOST_USER_UID=1000
ARG HOST_USER_GID=1000
RUN existing_user=$(getent passwd ${HOST_USER_UID} | cut -d: -f1); \
    existing_group=$(getent group ${HOST_USER_GID} | cut -d: -f1); \
    if [ -n "$existing_group" ] && [ "$existing_group" != "admin" ]; then \
      groupmod -n admin "$existing_group"; \
    elif [ -z "$existing_group" ]; then \
      groupadd -g ${HOST_USER_GID} admin; \
    fi; \
    if [ -n "$existing_user" ] && [ "$existing_user" != "admin" ]; then \
      usermod -l admin -d /home/admin -m "$existing_user"; \
    elif [ -z "$existing_user" ]; then \
      useradd -m -u ${HOST_USER_UID} -g ${HOST_USER_GID} -s /bin/bash admin; \
    fi && \
    echo "admin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG WMX3_SDK_PATH=/opt/wmx3
RUN mkdir -p ${WMX3_SDK_PATH} && \
    echo "${WMX3_SDK_PATH}/lib" > /etc/ld.so.conf.d/wmx3.conf
ENV WMX3_SDK_PATH=${WMX3_SDK_PATH}
ENV LD_LIBRARY_PATH=${WMX3_SDK_PATH}/lib:${LD_LIBRARY_PATH}
ENV PATH=${WMX3_SDK_PATH}/bin:${PATH}
