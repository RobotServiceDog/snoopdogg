FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# Basic dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion sudo git curl wget vim less \
    build-essential cmake python3-pip \
    python3-colcon-common-extensions python3-vcstool \
    python3-rosdep python3-argcomplete ccache \
    ros-humble-rmw-cyclonedds-cpp \ 
    ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py \
 && rm -rf /var/lib/apt/lists/*

# Create non-root user (robust if UID/GID already exist)
ARG DEVUSER=dev
ARG UID=1000
ARG GID=1000
RUN set -eux; \
    # Ensure a group with desired GID exists (name it DEVUSER if we create it)
    if ! getent group "${GID}" >/dev/null; then \
        groupadd -g "${GID}" "${DEVUSER}"; \
    fi; \
    # Create or align user to requested UID/GID and name
    if ! id -u "${UID}" >/dev/null 2>&1; then \
        useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${DEVUSER}"; \
    else \
        EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1)"; \
        usermod -l "${DEVUSER}" -d "/home/${DEVUSER}" -m "${EXISTING_USER}"; \
        usermod -g "${GID}" "${DEVUSER}"; \
    fi; \
    usermod -aG sudo "${DEVUSER}"; \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-nopasswd; \
    chmod 0440 /etc/sudoers.d/90-nopasswd

# Workspace + rosdep cache (owned by dev so rosdep doesn't need sudo)
RUN mkdir -p /snoopy/src /var/cache/rosdep \
 && chown -R ${UID}:${GID} /snoopy /var/cache/rosdep

WORKDIR /snoopy

# ROS env in dev's shell
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${DEVUSER}/.bashrc && \
    echo "source /snoopy/install/setup.bash 2>/dev/null || true" >> /home/${DEVUSER}/.bashrc && \
    echo "export COLCON_LOG_LEVEL=INFO" >> /home/${DEVUSER}/.bashrc && \
    echo "source /usr/share/bash-completion/bash_completion" >> /home/${DEVUSER}/.bashrc

# Initialize rosdep once (ignore if already done)
RUN rosdep init || true

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Run as dev (keeps sudo group; don't override with compose `user:`)
USER ${DEVUSER}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
