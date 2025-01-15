#!/bin/bash

gnome-terminal --tab -- bash -c "cd ~/catkin_folder/catkin_ws_VINS-with-Error-Model/;
				source ./devel/setup.bash;
				roscore;
				exec bash"

sleep 2

gnome-terminal --tab -- bash -c "cd ~/catkin_folder/catkin_ws_VINS-with-Error-Model/;
				source ./devel/setup.bash;
				roslaunch estimator euroc.launch;
				exec bash"

gnome-terminal --tab -- bash -c "cd ~/catkin_folder/catkin_ws_VINS-with-Error-Model/;
				source ./devel/setup.bash;
				roslaunch estimator rviz.launch;
				exec bash"

# gnome-terminal --tab -- bash -c "cd ~/catkin_folder/catkin_ws_VINS-with-Error-Model/;
# 				source ./devel/setup.bash;
# 				roslaunch benchmark_publisher publish.launch  sequence_name:=V2_03_difficult;
# 				exec bash"

gnome-terminal --tab -- bash -c "cd ~/catkin_folder/catkin_ws_VINS-with-Error-Model/;
				source ./devel/setup.bash;
				rosbag play /home/Jett64/catkin_folder/data/EuRoC/MH_01_easy.bag -s 0 -r 1;
				exec bash"
				