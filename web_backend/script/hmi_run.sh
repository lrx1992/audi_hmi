#!/bin/bash
# ps aux | grep roscore |awk '{print $2}' |xargs kill


CLEAR_CONTAINERS="docker rm -f \$(docker ps -a -q)"
GNSS_COMMAND="docker run -p 11888:11888 hmi_v3.22"
IMU_COMMAND="cd ~/software/matrix-web ; ./init.sh"
#CAR_COMMAND="docker run -it --rm -p 5580:5580 -p 8900:8900 -p 123:123 -p 6666:6666 navnet_server:v2 sh -c \"bash demo.sh 10 39 136\""
CAR_COMMAND="docker run -it --rm -p 5580:5580 -p 8900:8900 -p 123:123 -p 6666:6666 navnet_server:v1.2"

gnome-terminal	--window -e "bash -c '$CLEAR_CONTAINERS';bash" \
	--tab -e "bash -c 'sleep 1; $GNSS_COMMAND';bash" \
	--tab -e "bash -c 'sleep 1; $IMU_COMMAND';bash" \
        --tab -e "bash -c 'sleep 1; $CAR_COMMAND';bash" 

