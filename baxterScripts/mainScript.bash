#!/bin/bash

docker=0
haptic=0
vision=0
tf=0
bridge=0

while getopts d:h:b:v:t flag
do
    case "${flag}" in
        d) docker=1;;
        h) haptic=1;;
        v) vision=1;;
        t) tf=1;;
        b) bridge=1;;
    esac
done

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

cd ./scripts

if [ $1 = "all" ];
then

    echo "Running all scripts";
    
    source ./*.bash

else

    if [ $docker -eq 1 ];
    then

        source ./dockerScript.bash
    fi

    if [ $haptic -eq 1 ];
    then

        source ./ros2Script.bash
    fi

    if [ $bridge -eq 1 ];
    then

        source ./bridgeScript.bash
    fi

    if [ $vision -eq 1 ];
    then

        source ./visionScript.bash
    fi

    if [ $tf -eq 1 ];
    then

        source ./tfScript.bash
    fi

fi



