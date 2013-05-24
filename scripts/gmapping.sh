#!/bin/bash

gnome-terminal -x ./scripts/launch_gmapping.sh $1

gnome-terminal -x ./scripts/launch_rviz_gmapping.sh $1