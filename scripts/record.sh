#/bin/bash


if [ -n $1 ]; then
	SESSION="-o $1"
fi

rosbag record -q --split=1024 $SESSION scan narrow_scan odom diagnostics cmd_vel map