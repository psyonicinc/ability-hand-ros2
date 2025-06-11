ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    x11-apps \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Manus Dependencies
RUN apt-get update && apt-get install -y \
  build-essential \
  git \
  libtool \
  libzmq3-dev \
  zlib1g-dev \
  libusb-1.0-0-dev \
  libudev-dev \
  libncurses5-dev \
  gdb \
  && apt-get clean

ENV GRPC_RELEASE_TAG="v1.28.1"

RUN git clone -b ${GRPC_RELEASE_TAG} https://github.com/grpc/grpc /var/local/git/grpc && \
    cd /var/local/git/grpc && \
    git submodule update --init --recursive

RUN echo "-- installing protobuf" && \
    cd /var/local/git/grpc/third_party/protobuf && \
    ./autogen.sh && ./configure --enable-shared && \
    make -j$(nproc) && make -j$(nproc) check && make install && make clean && ldconfig

RUN echo "-- installing grpc" && \
    cd /var/local/git/grpc && \
    make -j$(nproc) && make install && make clean && ldconfig

# Install pip requirements
RUN python3 -m pip install ability-hand numpy==1.24 scipy==1.15

# Create workspace
WORKDIR /src
COPY ./src /src

# Source ROS & build
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]