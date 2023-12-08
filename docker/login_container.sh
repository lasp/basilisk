#!/bin/sh

. docker_config.sh
execute "docker exec -it -u root $DOCKER_CONTAINER_NAME /bin/bash"
