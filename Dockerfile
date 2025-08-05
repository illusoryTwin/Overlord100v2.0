# Use Ubuntu 22.04 as the base image
FROM osrf/ros:humble-desktop-full

# Set arguments for user creation
ARG USERNAME=mobile
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install gazebo
RUN apt-get update && apt-get install -y curl && \
    curl -sSL http://get.gazebosim.org | sh

# Install additional ROS 2 packages including development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-suite \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rqt-robot-steering \
    ros-humble-ros-gz \
    ros-humble-nav2-msgs \
    ros-humble-pcl-conversions \
    ros-humble-nav2-map-server \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-controller-manager \
    ros-dev-tools \
    x11-apps \
    xauth \
    --fix-missing

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Install colcon and necessary extensions using pip
# RUN pip3 install -U colcon-common-extensions

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up workspace  
# WORKDIR /home/ws/src
# COPY . /home/ws/src

# Copy startup script and set permissions
# COPY startup.bash /home/ws/startup.bash
# RUN chmod +x /home/ws/startup.bash

# Set the default user
USER $USERNAME

# Set the entry point
CMD ["bash"]
