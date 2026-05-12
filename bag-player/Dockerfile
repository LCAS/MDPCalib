ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-latest
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /workspace

# Copy the package manifest so rosdep can install all declared dependencies.
COPY package.xml /workspace/src/mdpcalib_bag_player/package.xml

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install --from-paths /workspace/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Python packages required for KITTI → ros2bag conversion (no rosdep keys available)
RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
        python3 \
        python3-pip \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
    && rm -rf /var/lib/apt/lists/*

# Pin numpy<2: pykitti (and several ros2 packages) are compiled against
# NumPy 1.x ABI and raise "cannot be run in NumPy 2.x" at runtime otherwise.
RUN pip3 install --no-cache-dir "numpy<2" pykitti progressbar2

COPY play_bag.sh play_kitti.sh kitti_to_ros2bag.py /workspace/
RUN chmod +x /workspace/play_bag.sh /workspace/play_kitti.sh

CMD ["/workspace/play_bag.sh"]
