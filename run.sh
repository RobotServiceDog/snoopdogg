#!/usr/bin/env bash
set -e

# Export your host UID/GID to map permissions correctly
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

# Default: simulation off
SIM_MODE="OFF"
if [[ "$1" == "--sim" ]]; then
  SIM_MODE="ON"
fi

docker compose build \
  --build-arg UID=$HOST_UID \
  --build-arg GID=$HOST_GID \
  --build-arg SIM_MODE=$SIM_MODE

docker compose run --rm \
  -e UID=$HOST_UID \
  -e GID=$HOST_GID \
  -e SIM_MODE=$SIM_MODE \
  --name snoopdogg_container \
  ros2 bash
