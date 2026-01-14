FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

# Basic tools + ROS deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-image-tools \
    ros-humble-ur-robot-driver \
    ros-humble-rviz2 \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/ros2_ws

# Copy only src first (better docker caching)
COPY ros2_ws/src /root/ros2_ws/src

# Build
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select camera_click_teleop

# Default: open a bash with ROS + workspace sourced
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && bash"]
