#!/bin/bash

source /home/arpl/devon_ws/catkin_ws/devel/setup.bash
folder="/home/arpl/devon_ws/data"


for power in {2..4};
do 
	for precision in FP16 INT8
	do
		# "$(rospack find ros_deep_learning)"/src/setpowermode $power #makes setpowermode within launch file to be redundant, but is necessary for the 10 second power mode switching delay.
		# sleep 10.0
		roslaunch ros_deep_learning depthnetexperiment.ros1.launch power_mode:=${power} precision:=${precision} image_folder:=${folder} publish_rate:=60

	done
done
