FROM nvcr.io/nvidia/cudagl:11.4.2-base-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
	gcc g++ wget curl vim git python3 python3-pip gnupg2 lsb-release

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-noetic-desktop-full

# Dependencies for virtual desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    lxde x11vnc xvfb mesa-utils supervisor \
    && apt-get purge -y light-locker

# Add a docker user so we that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/$USER/" > /etc/fixuid/config.yml

# Copy over configuration files
COPY rootfs /

# Add clearpath repo for jackal packages
# http://wiki.ros.org/ClearpathRobotics/Packages
RUN curl -s https://packages.clearpathrobotics.com/public.key | apt-key add - \
    && sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Install Jackal packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-jackal-simulator ros-noetic-jackal-navigation ros-noetic-jackal-desktop

# Install velodyne packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-velodyne-simulator ros-noetic-velodyne-description

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-rosdep

RUN sudo rosdep init && \
    rosdep update

USER docker

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/fixuid"]
CMD ["supervisord"]
