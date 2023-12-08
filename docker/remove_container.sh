#!/bin/sh

. docker_config.sh
execute "docker rm $DOCKER_CONTAINER_NAME"
execute "docker ps -a"
