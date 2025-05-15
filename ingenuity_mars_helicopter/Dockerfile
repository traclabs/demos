FROM osrf/space-ros:jazzy-2025.01.0

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DISTRO=jazzy
ENV HOME=/home/spaceros-user

# Install dependencies
# Related to space-ros/space-ros#195
RUN sudo apt update && \
    sudo apt install -y \
      ros-${ROS_DISTRO}-control-msgs \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Prepare the workspace
SHELL ["bash", "-c"]
RUN mkdir -p ${HOME}/ingenuity_ws/src
WORKDIR ${HOME}/ingenuity_ws

COPY ./ingenuity_demo src/ingenuity_demo
COPY .defaults.yaml .defaults.yaml

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source "${SPACEROS_DIR}"/install/setup.bash \
    && colcon build --symlink-install

# Source the workspace
RUN echo "source ${HOME}/ingenuity_ws/install/setup.bash" >> ${HOME}/.bashrc
