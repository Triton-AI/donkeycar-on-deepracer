#!/bin/bash

XSOCK=/tmp/.X11-unix
docker run --name dk-on-dr -it --rm \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
 --mount type=volume,source=dk-on-dr-dev,dst=/root \
haoru233/donkey-on-dr:dev