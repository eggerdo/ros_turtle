#!/bin/bash

if [[ -z $1 ]]; then
	echo "missing parameter"
	exit 1
fi

if [[ -z $2 ]]; then
	echo "missing parameter"
	exit 1
fi

rosbag filter $1 $2 'topic in ["odom","scan","imu_diagnose"] or (topic == "tf" and not ((m.transforms[0].header.frame_id == "/odom" and m.transforms[0].child_frame_id == "/base_footprint") or (m.transforms[0].header.frame_id == "/map" and m.transforms[0].child_frame_id == "/odom")))'