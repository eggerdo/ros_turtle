#!/bin/bash

gnome-terminal -x ./connect.sh

read -n 1 -s

gnome-terminal -x ./scripts/kinect_rviz_turtle.sh