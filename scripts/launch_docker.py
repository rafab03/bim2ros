xhost +local:docker
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --name nordic \
    --net=host \
    --privileged \
    --mount type=bind,source=$HOME/nordic_shared,target=/home/rva_container/rva_exchange \
    smarnav/nordic \
    bash
    
docker rm nordic
