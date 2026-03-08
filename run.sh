#!/usr/bin/env bash
set -e

# Export host UID/GID
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

# Defaults
SIM_MODE="OFF"
ATTACH_MODE="OFF"

# Parse arguments
for arg in "$@"; do
  case $arg in
    --sim)
      SIM_MODE="ON"
      ;;
    --attach)
      ATTACH_MODE="ON"
      ;;
  esac
done

# Fixed container name
CONTAINER_NAME="snoopdogg_container"

# Build Docker
docker compose build \
  --build-arg UID=$HOST_UID \
  --build-arg GID=$HOST_GID \
  --build-arg SIM_MODE=$SIM_MODE

# Check if container exists and is running
RUNNING_CONTAINER=$(docker ps -q -f name=$CONTAINER_NAME)

if [[ -n "$RUNNING_CONTAINER" ]]; then
    echo "Attaching to running container $CONTAINER_NAME..."
    docker exec -it $CONTAINER_NAME bash
else
    # If container exists but stopped, remove it
    EXISTING_CONTAINER=$(docker ps -aq -f name=$CONTAINER_NAME)
    if [[ -n "$EXISTING_CONTAINER" ]]; then
        echo "Removing old container $CONTAINER_NAME..."
        docker rm $CONTAINER_NAME
    fi

    # Run a new container
    echo "Starting new container $CONTAINER_NAME..."
    docker compose run --rm \
        -e UID=$HOST_UID \
        -e GID=$HOST_GID \
        -e SIM_MODE=$SIM_MODE \
        --name $CONTAINER_NAME \
        ros2 bash
fi