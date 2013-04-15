#!/bin/bash

if [ $1 -eq 1 ]; then
	. ./scripts/set_localhost.sh
fi

rosrun rviz rviz -d kinect_demo.vcg