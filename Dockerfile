FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    x11-apps \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install pip requirements
RUN python3 -m pip install ability-hand

# Create workspace
WORKDIR /src
COPY ./src /src

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