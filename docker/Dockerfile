# syntax=docker/dockerfile:1.7
FROM osrf/space-ros:jazzy-2025.10.0

ARG DEBIAN_FRONTEND=noninteractive

# Space ROS in this image expects CycloneDDS; avoid FastDDS default selection from /opt/ros/jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DISTRO=jazzy
ENV USERNAME=spaceros-user
ENV HOME=/home/spaceros-user
ENV GZ_VERSION=harmonic
ENV COLCON_LOG_PATH=/home/spaceros-user/.colcon/log

USER root
# Base tools needed to add external apt repositories.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      bash ca-certificates \
      git python3-vcstool python3-yaml \
      lsb-release wget gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg\
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install SpaceTry runtime + build dependencies in one layer to avoid
# repeated apt metadata downloads on cold builds.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      build-essential \
      gz-${GZ_VERSION} \
      ros-${ROS_DISTRO}-actuator-msgs \
      ros-${ROS_DISTRO}-behaviortree-cpp \
      ros-${ROS_DISTRO}-builtin-interfaces \
      ros-${ROS_DISTRO}-control-msgs \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-gz-ros2-control \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-rcl-interfaces \
      ros-${ROS_DISTRO}-rclcpp \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-rosbag2 \
      ros-${ROS_DISTRO}-sdformat-urdf \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Make every login shell (bash -l / bash -lc) see BOTH Jazzy (apt) and Space ROS
RUN tee /etc/profile.d/spacetry_ros_env.sh >/dev/null <<'EOF' && chmod 0644 /etc/profile.d/spacetry_ros_env.sh
# SpaceTry: ensure ROS Jazzy underlay + Space ROS overlay are always present
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi
if [ -f /opt/ros/spaceros/setup.bash ]; then
  . /opt/ros/spaceros/setup.bash
fi
EOF

RUN cat >/etc/profile.d/spacetry_deps_link.sh <<'EOF'
# SpaceTry: expose pinned Space ROS demos in /ws/src for colcon + ros2
if [ -d /opt/spacetry_deps/space_ros_demos ]; then
  mkdir -p /ws/src
  if [ ! -e /ws/src/space_ros_demos ]; then
    ln -s /opt/spacetry_deps/space_ros_demos /ws/src/space_ros_demos
  fi
fi
EOF

# prepare workspace dirs explicitly
RUN mkdir -p /ws/src

WORKDIR /ws

# Copy build inputs in cache-friendly order.
# Dependency manifests and docker scripts rarely change compared to src/,
# so this keeps external fetch layers warm across normal code edits.
COPY --chown=${USERNAME}:${USERNAME} deps /ws/deps
COPY --chown=${USERNAME}:${USERNAME} docker /ws/docker

# Remove any stale colcon artifacts that slipped past .dockerignore
# (prevents broken package.dsv files from a previous host build)
RUN rm -rf /ws/build /ws/install /ws/log
RUN install -d -o ${USERNAME} -g ${USERNAME} /ws/build /ws/install /ws/log ${COLCON_LOG_PATH}

# Add GitHub host key to known_hosts to avoid interactive prompt
RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Import pinned Space ROS demos.
# When SSH agent is available (local dev), uses git@github.com URLs from spacetry.repos.
# When not (CI), rewrites SSH URLs to HTTPS for public repos.
RUN --mount=type=ssh,required=false \
    mkdir -p /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts 2>/dev/null && \
    if ! ssh -o BatchMode=yes -T git@github.com 2>&1 | grep -q 'successfully authenticated'; then \
      git config --global url."https://github.com/".insteadOf "git@github.com:"; \
    fi && \
    rm -rf /ws/src/space_ros_demos && \
    mkdir -p /ws/src && \
    vcs import /ws/src < /ws/deps/spacetry.repos && \
    bash /ws/docker/patch_imported_space_ros_demos.sh /ws/src/space_ros_demos

# Cache imported deps outside /ws so bind-mounting /ws won't hide them.
# Permanently ignore the duplicate packages inside the cached copy.
RUN mkdir -p /opt/spacetry_deps && \
    rm -rf /opt/spacetry_deps/space_ros_demos && \
    cp -a /ws/src/space_ros_demos /opt/spacetry_deps/space_ros_demos && \
    # Ignore the entire racs2 subtree (it contains a duplicate curiosity_rover_demo)
    touch /opt/spacetry_deps/space_ros_demos/racs2_demos_on_spaceros/COLCON_IGNORE && \
    # And also ignore the duplicate package directory itself (belt + suspenders)
    touch /opt/spacetry_deps/space_ros_demos/racs2_demos_on_spaceros/curiosity_rover_demo/COLCON_IGNORE && \
    # CRITICAL: remove the workspace copy inside the image to prevent duplicates
    rm -rf /ws/src/space_ros_demos

# Ignore all top-level demos except Curiosity rover (keeps SpaceTry build small and deterministic)
RUN for d in /opt/spacetry_deps/space_ros_demos/*; do \
      bn="$(basename "$d")"; \
      if [ -d "$d" ] && [ "$bn" != "curiosity_rover" ]; then \
        touch "$d/COLCON_IGNORE"; \
      fi; \
    done && \
    # Also ignore the racs2 subtree inside cached demos (duplicate package names / extra deps)
    touch /opt/spacetry_deps/space_ros_demos/racs2_demos_on_spaceros/COLCON_IGNORE

# Ensure runtime user can read cached deps and work in /ws.
RUN chown -R ${USERNAME}:${USERNAME} /opt/spacetry_deps /ws
RUN install -m 0644 /ws/docker/spacetry_ws_overlay.sh /etc/profile.d/spacetry_ws_overlay.sh

# Copy the workspace sources only after dependency import so normal code edits
# do not force a re-clone of the external demos repository.
COPY --chown=${USERNAME}:${USERNAME} src /ws/src

# Add runtime user to video and render groups for GPU device access (/dev/dri)
# Use host's render GID (default 109) so container user can access /dev/dri/renderD*
ARG RENDER_GID=109
RUN groupadd -f -g ${RENDER_GID} render && usermod -a -G video,render ${USERNAME}

# Drop privileges for runtime
USER ${USERNAME}
WORKDIR /ws

# Build the workspace into the image so first-run users do not need a manual colcon step.
# Use a per-user log directory to avoid failures when /ws/log is bind-mounted from the host.
RUN bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && colcon build --merge-install --event-handlers console_direct+"

ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["bash", "-lc", "echo SpaceTry container OK && ls -la"]
