#!/bin/bash

cd scripts

gnome-terminal -t "ros_server" -x bash -c "roslaunch information_map sim_env.launch;exec bash;"

sleep 1s

python3 info_map_gen.py