# F1tenth

## F1tenth competition

## BRASS demo

## Adaptive control using Caffe

## Using Slam with lidar:
This is for sending the raw data from lidar to the hector_slam node and generate map for surrounding enviroment. If you want to replay a ros_bag, please change the parameter "/use_sim_time" to Ture in tutorial.launch.

http://f1tenth.org/lab_instructions/W3_T1_Using%20the%20Hector%20SLAM.pdf

Please following the above link to set Slam up.Then first use the following command to go to the directory:

roscd hector_slam_launch/launch/

replace the tutorial.launch with the one in the Slam_launch folder.

Then use the command to another directory:

roscd hector_mapping/launch

repalce the mapping_defualt.launch with the one in the Slam_launch folder.
