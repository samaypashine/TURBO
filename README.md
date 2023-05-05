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


