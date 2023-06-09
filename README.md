# TURBO

TURBO (Twin Universal Robot Boosting Operation). This is the project dealing with using two Universal Robot 5 arms at the same time synchronously. I created this ROS package with all the specifics including the XACRO files, Controller Files, and tested this package by moving the arm to multiple waypoints interchangably and at the same time. To use this package, please follow the steps below.


## Requirements

1. Ubuntu 18.04
2. ROS Melodic
3. Run `chmod +x ./install.sh` after going in the repository to install the dependenies.
4. Run `./install.sh`
5. Also Run, `rosdep install --from-path src --ignore-src -y` from you ROS workspace.
6. Run `git clone https://github.com/samaypashine/Universal_Robots_ROS_Driver` in the ROS workspace.
7. Run `git clone https://github.com/samaypashine/universal_robot` in the ROS workspace.


## Calibration Steps [ OPTIONAL ]
1. ``` roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:=<path including the .yaml filename> ```


## Steps to Use
1. ``` catkin_make ```
2. ``` roslaunch turbo_bringup turbo.launch ``` // Double Tab to look at the parameters. 
3. Once the Rviz is booted, open the `rviz_config.rviz` file to start planning and moving the arms using MoveIt.

## Demos
1. Teleoperation Demo : In this demo, both arms can be controlled using (w, s, a, d) for right arm, and (i, j, k, l) for left arm at the same time using URscripts.
   ``` rosrun turbo_demos teleoperation```
2. Pick n Place demo : In this demo, both arms would move to a location pick an object, Place it to a location for the other robot to do the same.
   ``` rosrun turbo_demos pick_n_place```

## Teleoperation Demonstration
   <p align="center"> 
      <img src="https://github.com/samaypashine/TURBO/assets/51475380/eb6d0b7f-4194-4f0c-a749-93bcc9cb6742" />
   </p>
