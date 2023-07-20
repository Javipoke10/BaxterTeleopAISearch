#!/bin/bash

echo "Docker running";
sudo docker run -ti --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --env QT_X11_NO_MITSHM=1 gscar:PhantomOmni