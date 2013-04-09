#!/bin/bash

gnome-terminal -x ./scripts/kinect_roscore.sh

sleep 2

gnome-terminal -x ./scripts/kinect_scan.sh

gnome-terminal -x ./scripts/kinect_rviz_local.sh