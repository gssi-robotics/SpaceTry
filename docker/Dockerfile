FROM osrf/space-ros:jazzy-2025.10.0

ARG DEBIAN_FRONTEND=noninteractive

USER root

# Base tools + ros_gz
 RUN apt-get update && apt-get install -y --no-install-recommends \
    bash ca-certificates \
    git python3-vcstool \
    ros-jazzy-ros-gz \
    lsb-release wget gnupg \
 && rm -rf /var/lib/apt/lists/*

# Add OSRF Gazebo apt repo (for gz-harmonic)
RUN wget -qO- https://packages.osrfoundation.org/gazebo.gpg \
    | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Harmonic (provides `gz`)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

USER spaceros-user

WORKDIR /ws
COPY --chown=spaceros-user:spaceros-user . /ws
RUN vcs import /ws/src < /ws/deps/marti.repos

# Keep a copy of imported deps outside /ws so bind-mounting /ws won't hide them
RUN mkdir -p /opt/marti_deps && \
    rm -rf /opt/marti_deps/space_ros_demos && \
    cp -a /ws/src/space_ros_demos /opt/marti_deps/space_ros_demos

ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["bash", "-lc", "echo MARTI container OK && ls -la"]
