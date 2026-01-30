FROM osrf/space-ros:jazzy-2025.10.0

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DISTRO=jazzy
ENV HOME=/home/spaceros-user

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Install dependencies
# Related to space-ros/space-ros#195 and space-ros/space-ros#196
RUN sudo apt update && \
    sudo apt install -y \
      ros-${ROS_DISTRO}-control-msgs \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Prepare the workspace
SHELL ["bash", "-c"]
RUN mkdir -p ${HOME}/lunar_terrain_ws/src
WORKDIR ${HOME}/lunar_terrain_ws

#COPY ./lunar_terrain_demo src/lunar_terrain_demo
COPY .defaults.yaml .defaults.yaml

# Build the workspace
RUN source "${SPACEROS_DIR}"/setup.bash \
    && colcon build

# Source the workspace
RUN echo "source ${HOME}/lunar_terrain_ws/install/setup.bash" >> ${HOME}/.bashrc
