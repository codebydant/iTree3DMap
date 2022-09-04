# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=`pwd`:/tmp/project \
    docker.io/library/itreemap:1.0-ubuntu22.04
# Disallow X server connection
xhost -local:root