. docker_config

xhost +

# XAUTH=/tmp/.docker.xauth
# if [ ! -f $XAUTH ]
# then
#     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
#     if [ ! -z "$xauth_list" ]
#     then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     else
#         touch $XAUTH
#     fi
#     chmod a+r $XAUTH
# fi

docker run \
    -v $(pwd)/code:/home/user/code \
    --rm \
    -it \
    --network host \
    --privileged \
    --device /dev/dri \
    --device /dev/snd \
    --env="QT_X11_NO_MITSHM=1" \
    --runtime=nvidia \
    -v /run/user/1000/pulse:/run/user/1000/pulse \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) \
    -e DISPLAY=$DISPLAY \
    --name $CONTAINER_NAME \
    python3 /home/user/code/add_armod_tf.py                           
