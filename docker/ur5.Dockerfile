FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Install MoveIt, UR driver, and development tools
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    curl \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-moveit \
    ros-noetic-moveit-commander \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-tf2-tools \
    ros-noetic-rviz \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-controller-manager \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-universal-robots \
    ros-noetic-ur-client-library \
    ros-noetic-ur-robot-driver \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws

# Initialize rosdep if needed
RUN rosdep update || true

# Build empty workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# Auto-source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /catkin_ws

CMD ["/bin/bash"]
