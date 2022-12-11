#!/bin/bash
#
FCUURL=$1
STARTUP_DELAY=$2

printf "Sleeping for $STARTUP_DELAY seconds...\n"
sleep $STARTUP_DELAY

source /opt/ros/melodic/setup.bash
roscd mavros

roslaunch mavros px4.launch fcu_url:=${FCUURL} 2>&1 > /mavros.log &