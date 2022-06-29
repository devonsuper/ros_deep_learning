#!/bin/bash

source /home/arpl/devon_ws/catkin_ws/devel/setup.bash
folder="/home/arpl/devon_ws/experiment/images"


for power in {2..4};
do 
	for precision in FP16 INT8
	do

		mkdir "${folder}/${precision}_power${power}"

		roslaunch ros_deep_learning depthnetonfile.ros1.launch power_mode:=${power} precision:=${precision} image_folder:=${folder}

	done
done
