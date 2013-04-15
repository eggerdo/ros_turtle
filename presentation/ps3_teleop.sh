#!/bin/bash

gnome-terminal -x sudo `rospack find ps3joy`/ps3joy.py &

gnome-terminal -x ./scripts/launch_ps3_teleop.sh