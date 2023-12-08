#!/bin/sh

# Create the docker container with a bind mount:
echo "Creating container..."
. docker_config.sh
execute "docker run -d \
  --name $DOCKER_CONTAINER_NAME \
  --mount type=bind,source=\"$(pwd)\"/../..,target=/share \
  $DOCKER_IMAGE_NAME \
  sleep infinity"

echo "Finished creating container \"$DOCKER_CONTAINER_NAME\"."
execute "docker ps -a"

echo ""
echo "Run ./login_container.sh to log in."
