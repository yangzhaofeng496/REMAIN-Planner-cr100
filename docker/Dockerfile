FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/opt/remani_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    libeigen3-dev \
    libompl-dev \
    libopencv-dev \
    python3-catkin-tools \
    qtbase5-dev \
    ros-noetic-cmake-modules \
    ros-noetic-control-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-geometry-msgs \
    ros-noetic-image-transport \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-nav-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-roscpp \
    ros-noetic-roslib \
    ros-noetic-rospy \
    ros-noetic-rviz \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    ros-noetic-trajectory-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen \
 && ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported

RUN mkdir -p "${CATKIN_WS}/src"
WORKDIR ${CATKIN_WS}/src

COPY . ${CATKIN_WS}/src/REMANI-Planner

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY docker/ros_entrypoint.sh /ros_entrypoint_remani.sh
RUN chmod +x /ros_entrypoint_remani.sh

ENTRYPOINT ["/ros_entrypoint_remani.sh"]
CMD ["bash"]
