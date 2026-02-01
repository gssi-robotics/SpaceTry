FROM osrf/space-ros:jazzy-2025.10.0

ARG DEBIAN_FRONTEND=noninteractive

USER root

# Base tools + ros_gz + vcstool + git
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

# prepare workspace dirs explicitly
RUN mkdir -p /ws/src

WORKDIR /ws

# Copy repo into image. Chown so runtime user can work in /ws when not bind-mounted.
COPY --chown=spaceros-user:spaceros-user . /ws

# Import pinned Space ROS demos into the image workspace.
RUN rm -rf /ws/src/space_ros_demos && \
    mkdir -p /ws/src && \
    vcs import /ws/src < /ws/deps/marti.repos

# Cache imported deps outside /ws so bind-mounting /ws won't hide them.
# Also permanently ignore the duplicate curiosity_rover_demo package in the cached copy.
RUN mkdir -p /opt/marti_deps && \
    rm -rf /opt/marti_deps/space_ros_demos && \
    cp -a /ws/src/space_ros_demos /opt/marti_deps/space_ros_demos && \
    mkdir -p /opt/marti_deps/space_ros_demos/racs2_demos_on_spaceros/curiosity_rover_demo && \
    touch /opt/marti_deps/space_ros_demos/racs2_demos_on_spaceros/curiosity_rover_demo/COLCON_IGNORE

# Ensure runtime user can read cached deps and work in /ws.
RUN chown -R spaceros-user:spaceros-user /opt/marti_deps /ws

# Drop privileges for runtime
USER spaceros-user
WORKDIR /ws

ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["bash", "-lc", "echo MARTI container OK && ls -la"]
