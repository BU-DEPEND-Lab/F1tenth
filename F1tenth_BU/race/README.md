# Racecar Field Setup

## Install dependencies
    sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers
    
## Build Race package
    # Copy race folder to the src folder under current workspace
    cp -r ~/F1tenth_BU/race ~/catkin_ws/src/
    # Build the pacakges
    source /opt/ros/kinetic/setup.bash
    cd ..
    catkin_make
    source devel/setup.bash

## How to use google cartographer for SLAM

### 1) Install google cartographer and ROS integration
follow instruction [here](https://google-cartographer-ros.readthedocs.io/en/latest/) or commandlines below.

    # Install wstool and rosdep. 
    sudo apt-get update 
    sudo apt-get install -y python-wstool python-rosdep ninja-build 
    
    # Create a new workspace in 'slam_ws'. 
    mkdir slam_ws 
    cd slam_ws 
    wstool init src 
    
    # Merge the cartographer_ros.rosinstall file and fetch code for dependencies. 
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall 
    wstool update -t src 
    
    # Install proto3. src/cartographer/scripts/install_proto3.sh 
    # Install deb dependencies.  
    # The command 'sudo rosdep init' will print an error if you have already  
    # executed it since installing ROS. This error can be ignored. 
    sudo rosdep init 
    rosdep update 
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 
    
    # Build and install. 
    catkin_make_isolated --install --use-ninja 
    source install_isolated/setup.bash

### 2) Build a map and save it for localization later
Substitute with **[mapping.launch](https://github.com/JmfanBU/F1tenth_BU/blob/master/slam/mapping.launch)** and **[demo\_backpack\_2d\_localization.launch](https://github.com/JmfanBU/F1tenth_BU/blob/master/slam/demo_backpack_2d_localization.launch)** in cartographer\_ros launch directory. Add **[mapping.lua](mapping.lua)** to configuration_files directory.

    # Substitute launch files
    cp ~/F1tenth_BU/slam/mapping.launch ~/slam_ws/src/cartographer_ros/cartographer_ros/launch
    cp ~/F1tenth_BU/slam/demo_backpack_2d_localization.launch ~/slam_ws/src/cartographer_ros/cartographer_ros/launch
    
    # Add configuration files
    cp ~/F1tenth_BU/slam/mapping.lua ~/slam_ws/src/cartographer_ros/cartographer_ros/configuration_files
    
    # Rebuild the package
    catkin_make_isolated --install --use-ninja 
    source install_isolated/setup.bash
    
Start lidar first.

    # start eth0 for lidar
    sudo ifconfig eth0 192.168.1.10 netmask 255.255.255.0 up
    
    # make sure the lidar is spinning after the power wire is connected
    # connect the ethernet wire to the TX1 board
    # launch urg_node and the lidar starts to stream data
    roslaunch race lidar.launch
    
Start ***cartographer\_node*** by following commandline

      # start the mapping node
      roslaunch cartographer_ros mapping.launch
      
      # start rviz for visualization
      rosrun rviz rviz -d `rospack find cartographer_ros`/configuration_files/demo_2d.rviz
   
Use the joystick to manually control the car moving around to explore and build a map. The built map will be more precise if the car moves slowly and turns smoothly.

    # start to joystick control 
    roslaunch race joystick.launch

After you finish building the map, save the map into a **.pbstram** file for further use. 

      # call the /write_state service to save it
      # use your own map_file_path and map_file_name in the commandline
      rosservice call /write_state ~/map_file_path/map_file_name.bag.pbstream
### 3) Use the saved map for localization
Use the following commandline to localize the car

      # load the map by specifying the load_state_filename 
      roslaunch cartographer_ros demo_backpack_2d_localization.launch    load_state_filename:=${HOME}/map_file_name.bag.pbstream
      # start rviz for visualization
      rosrun rviz rviz -d `rospack find cartographer_ros`/configuration_files/demo_2d.rviz
 A **/tf** from *map\_frame* (typically map) to *published\_frame*(typically base_link) is broadcasted and you can listen to the transformation to get the car's location in *map\_frame*.

## How to use PID control to track waypoint
     # start the path_tracking.py
     roslaunch race path_tracking.launch

 - Press and hold the **RB** button and use two joysticks to manually control the car.
 - Press and hold button **B** and the car will automatically track the waypoints.
 - After the car finishes tracking waypoints, press button **A** to reset waypoints.
