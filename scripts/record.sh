#!/bin/bash

if [ -n "$1" ]; then
	SESSION="-o $1"
fi

rosbag record -q --split --size=1024 $SESSION scan narrow_scan odom diagnostics cmd_vel map tf imu_diagnose particlecloud move_base/TrajectoryPlannerROS/global_plan