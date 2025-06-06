FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
COPY ./ros2_ws /ros2_ws

# Source ROS & build
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Setup GUI forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/root/.Xauthority

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]