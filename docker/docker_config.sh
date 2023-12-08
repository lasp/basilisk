#!/bin/sh

DOCKER_CONTAINER_NAME="basilisk_riscv_container"
DOCKER_IMAGE_NAME="basilisk_riscv_container:latest"
export DOCKER_CONTAINER_NAME
export DOCKER_IMAGE_NAME

# Helper function to print out command as executed:
execute () {
  echo "$ $@"
  eval "$@"
}
