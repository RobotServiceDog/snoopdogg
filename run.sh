#!/usr/bin/env bash
set -e

# Export your host UID/GID to map permissions correctly
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

docker compose build \
  --build-arg UID=$HOST_UID \
  --build-arg GID=$HOST_GID

docker compose run --rm \
  -e UID=$HOST_UID \
  -e GID=$HOST_GID \
  ros2 bash
