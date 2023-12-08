#!/bin/sh

. docker_config.sh
execute "docker start $DOCKER_CONTAINER_NAME"
execute "docker ps -a"
