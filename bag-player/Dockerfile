ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-latest
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

USER root

WORKDIR /workspace

# Copy the package manifest so rosdep can install all declared dependencies.
COPY package.xml /workspace/src/mdpcalib_bag_player/package.xml

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install --from-paths /workspace/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

COPY play_bag.sh /workspace/play_bag.sh
RUN chmod +x /workspace/play_bag.sh

CMD ["/workspace/play_bag.sh"]
