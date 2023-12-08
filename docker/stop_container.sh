#!/bin/sh

. docker_config.sh
execute "docker stop $DOCKER_CONTAINER_NAME"
execute "docker ps -a"
