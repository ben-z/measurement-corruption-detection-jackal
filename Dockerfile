FROM ros:noetic-robot as robot

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc g++ wget curl vim git python3 python3-pip gnupg2 lsb-release

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

# Add clearpath repo for jackal packages
# http://wiki.ros.org/ClearpathRobotics/Packages
RUN curl -s https://packages.clearpathrobotics.com/public.key | apt-key add - \
    && sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Install Jackal packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-jackal-simulator ros-noetic-jackal-navigation ros-noetic-jackal-robot

# Install packages required by our stack. This can be done using rosdep, but pre-installing
# them here speeds up the build process.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-serial ros-noetic-um7 ros-noetic-hector-localization \
    ros-noetic-ros-numpy ros-noetic-roslint libceres-dev

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-rosdep tmux ros-noetic-foxglove-bridge \
    ros-noetic-tf2-tools lsof htop

USER docker

RUN pip install typeguard==3.0.0rc2 cvxpy==1.3.0 debugpy==1.6.6

# Copy over configuration files
COPY docker-rootfs/shared/ /
COPY docker-rootfs/robot/ /

# custom bashrc
RUN echo "source /etc/local.bashrc" >> ~/.bashrc

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/fixuid"]
CMD ["sleep", "infinity"]

FROM nvidia/cudagl:11.0-base-ubuntu20.04 as sim

ENV DEBIAN_FRONTEND=noninteractive

# Workaround for the nvidia key rotation issue:
# https://forums.developer.nvidia.com/t/notice-cuda-linux-repository-key-rotation/212771/10
RUN rm /etc/apt/sources.list.d/cuda.list

RUN apt-get update && apt-get install -y --no-install-recommends \
	gcc g++ wget curl vim git python3 python3-pip gnupg2 lsb-release

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-noetic-desktop-full

# Dependencies for virtual desktop
RUN wget -q https://sourceforge.net/projects/turbovnc/files/3.0.3/turbovnc_3.0.3_amd64.deb \
    && wget -q https://sourceforge.net/projects/virtualgl/files/3.0.2/virtualgl32_3.0.2_amd64.deb \
    && wget -q https://sourceforge.net/projects/virtualgl/files/3.0.2/virtualgl_3.0.2_amd64.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
        ./turbovnc_3.0.3_amd64.deb ./virtualgl32_3.0.2_amd64.deb ./virtualgl_3.0.2_amd64.deb \
        lxde mesa-utils supervisor \
    && apt-get purge -y light-locker \
    && rm -f turbovnc_3.0.3_amd64.deb virtualgl32_3.0.2_amd64.deb virtualgl_3.0.2_amd64.deb

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

# Add clearpath repo for jackal packages
# http://wiki.ros.org/ClearpathRobotics/Packages
RUN curl -s https://packages.clearpathrobotics.com/public.key | apt-key add - \
    && sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Install Jackal packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-jackal-simulator ros-noetic-jackal-navigation \
    ros-noetic-jackal-desktop ros-noetic-jackal-base
    
# Install packages required by our stack. This can be done using rosdep, but pre-installing
# them here speeds up the build process.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-serial ros-noetic-um7 ros-noetic-hector-localization \
    ros-noetic-ros-numpy ros-noetic-roslint libceres-dev

# Install velodyne packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-velodyne-simulator ros-noetic-velodyne-description

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-rosdep tmux ros-noetic-foxglove-bridge \
    ros-noetic-tf2-tools lsof htop

USER docker

RUN sudo rosdep init && \
    rosdep update

RUN pip install typeguard==3.0.0rc2 cvxpy==1.3.0 debugpy==1.6.6

# Required by TurboVNC
RUN touch ~/.Xauthority

# Copy over configuration files
COPY docker-rootfs/shared/ /
COPY docker-rootfs/sim/ /

# custom bashrc
RUN echo "source /etc/local.bashrc" >> ~/.bashrc

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/fixuid"]
CMD ["supervisord"]
