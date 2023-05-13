/**
 * @file pick_n_place.cpp
 * @author Samay Pashine
 * @brief This is a demo file to move both of the arm sequentially, using path planning from moveit.
 * @version 1.0
 * @date 2023-05-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>


#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <turbo_robot_vision/TabletopPerception.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPositionIK.h>

#include <robotiq_85_msgs/GripperCmd.h>


// Declaring the ROS Publisher, Service client, and other variables.
ros::Publisher pose_pub;

ros::Publisher left_gripper_pub;
ros::Publisher right_gripper_pub;

ros::ServiceClient ik_client;

geometry_msgs::PoseStamped left_arm_current_pose;
geometry_msgs::PoseStamped right_arm_current_pose;

moveit::planning_interface::MoveGroupInterface *left_arm_group;
moveit::planning_interface::MoveGroupInterface *right_arm_group;

double Z_coord = 0.704;
std::string frame_id = "world";


/**
 * @brief Function to block the execution temporarily.
 * 
 */
void pressEnter()
{
	std::cout << "************************************ Press the Any key to continue ************************************ \n";
	struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
}


/**
 * @brief Function to close the gripper of the arms on the basis of the arms name.
 * 
 * @return 0
 */
bool close_gripper(std::string arm_name)
{
	robotiq_85_msgs::GripperCmd msg;
	msg.position = 0.0;
	msg.speed = 1.0;
	msg.force = 100.0;

	if (arm_name.compare("left"))
		left_gripper_pub.publish(msg);
	else
		right_gripper_pub.publish(msg);

	ros::spinOnce();	
	return 0;
}


/**
 * @brief Function to open the gripper of the arms on the basis of the arms name.
 * 
 * @return 0
 */
bool open_gripper(std::string arm_name)
{	
	robotiq_85_msgs::GripperCmd msg;
	msg.position = 1.0;
	msg.speed = 1.0;
	msg.force = 100.0;

	if (arm_name.compare("left"))
		left_gripper_pub.publish(msg);
	else
		right_gripper_pub.publish(msg);

	ros::spinOnce();
	return 0;
}


/**
 * @brief Function to move the arm in the initial position using joint angles. The values MUST match
	or it will not be moving to the correct hard coded position.
 * 
 */
void home_pos(std::string arm_name)
{
	std::map<std::string, double> left_arm_home_pos;
	std::map<std::string, double> right_arm_home_pos;
	
	left_arm_home_pos["left_shoulder_pan_joint"] =  -0.02146755; 
	left_arm_home_pos["left_shoulder_lift_joint"] =  -2.02021861;
	left_arm_home_pos["left_elbow_joint"] =  -1.583014;
	left_arm_home_pos["left_wrist_1_joint"] = -0.97930424; 
	left_arm_home_pos["left_wrist_2_joint"] = -2.49791523; 
	left_arm_home_pos["left_wrist_3_joint"] = 0.05602507;

	right_arm_home_pos["right_shoulder_pan_joint"] =  -0.03017;
	right_arm_home_pos["right_shoulder_lift_joint"] =  -1.2833;
	right_arm_home_pos["right_elbow_joint"] =  1.6797;
	right_arm_home_pos["right_wrist_1_joint"] = -1.8945;
	right_arm_home_pos["right_wrist_2_joint"] = 2.4260;
	right_arm_home_pos["right_wrist_3_joint"] = 0.1295;
	
	ROS_INFO("Planning the Path to Home Position.");
	if (arm_name.compare("left"))
	{	
		left_arm_group->setJointValueTarget(left_arm_home_pos);

		ROS_INFO("Moving Left Arm to Home Positions.");
		left_arm_group->move();
	}
	else if (arm_name.compare("right"))
	{
		right_arm_group->setJointValueTarget(right_arm_home_pos);
		ROS_INFO("Moving Right Arm to Home Positions.");
		right_arm_group->move();
	}
	else
	{
		ROS_ERROR("Arm name INVALID.");
		exit(1);
	}
}



/**
 * @brief Function to move the object to any random position on the plane.
 * 
 * @param target_object 
 */
void move_to_position(float x, float y, float z, bool flag)
{
	char execution_flag;
	geometry_msgs::PoseStamped stampedPose;
	
	stampedPose.header.frame_id = "world";
	stampedPose.header.stamp = ros::Time(0);
	stampedPose.pose.position.x = x;
	stampedPose.pose.position.y = y;
	stampedPose.pose.position.z = z;
	stampedPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);

	ROS_INFO_STREAM(stampedPose);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	if (flag)
	{
		right_arm_group->setPoseTarget(stampedPose);
		right_arm_group->setStartStateToCurrentState();
		right_arm_group->setPoseReferenceFrame("world");
		
		bool success = (right_arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (success)
			right_arm_group->execute(my_plan);
	}
	else
	{
		left_arm_group->setPoseTarget(stampedPose);
		left_arm_group->setStartStateToCurrentState();
		left_arm_group->setPoseReferenceFrame("world");
		
		bool success = (left_arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (success)
			left_arm_group->execute(my_plan);
	}
}



/**
 * @brief Main Driver code. (Known optimal planners: RRT, PRM)
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
	// Intialize ROS with this node name
	ros::init(argc, argv, "turbo_pick_n_place_demo");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
    spinner.start();
    
	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_arm_demo/pose", 10);
	
	// Configuring the Left and Right Arm gripper publishers
	left_gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/left/gripper/cmd", 10);
	right_gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/right/gripper/cmd", 10);

	ROS_INFO("TURBO PicknPlace Demo");

	//Need to only call MoveGroupInterface ONCE in the program to allow the arms to accept targets, and move to them
	left_arm_group = new moveit::planning_interface::MoveGroupInterface("left_manipulator");
	right_arm_group = new moveit::planning_interface::MoveGroupInterface("right_manipulator");

	// Initializing the Left arm planning group
	left_arm_group->setPlannerId("RRTConfigDefault");
    left_arm_group->setGoalTolerance(0.10);
	left_arm_group->setPlanningTime(10);

	// Initializing the Right arm planning group
	right_arm_group->setPlannerId("TRRT");
    right_arm_group->setGoalTolerance(0.10);
	right_arm_group->setPlanningTime(10);

	// Initialize the ARM and open the gripper
	open_gripper("left");
	open_gripper("right");

	pressEnter();

	// Moving Left Arm to Home Position
	home_pos("left");

	// Moving Right Arm to Home Position
	home_pos("right");
	pressEnter();

	float right_x = 0.87593;
	float right_y = 0.01145;
	float right_z = 1.04981;

	float left_x = 0.425933;
	float left_y = 0.298069;
	float left_z = 1.06981;

	// Picking n Placing the object using Right Arm.
	// Move Right Arm Above the Object
	move_to_position(right_x, right_y, right_z, true);
	ros::Duration(1).sleep();

	// Lower the Right Arm
	move_to_position(right_x, right_y, right_z-0.1, true);
	ros::Duration(1).sleep();

	// Close the Gripper
	close_gripper("right");

	// Moving Up the Right Arm
	move_to_position(right_x, right_y, right_z+0.1, true);
	ros::Duration(1).sleep();

	// Move Right Arm to the Home position
	home_pos("right");
	ros::Duration(1).sleep();

	// Move Right Arm Above the Object
	move_to_position(right_x, right_y, right_z, true);
	ros::Duration(1).sleep();

	// Lower the Right Arm
	move_to_position(right_x, right_y, right_z-0.10, true);
	ros::Duration(1).sleep();

	// Close the Gripper
	open_gripper("right");
	
	// Moving Up the Right Arm
	move_to_position(right_x, right_y, right_z+0.10, true);
	ros::Duration(1).sleep();

	// Move Right Arm to the Home position
	home_pos("right");
	ros::Duration(1).sleep();


	// Picking n Placing the object using Left Arm.
	// Move Left Arm Above the Object
	move_to_position(left_x, left_y, left_z, false);
	ros::Duration(1).sleep();

	// Lower the Left Arm
	move_to_position(left_x, left_y, left_z-0.10, false);
	ros::Duration(1).sleep();

	// Close the Gripper
	close_gripper("left");

	// Moving Up the Left Arm
	move_to_position(left_x, left_y, left_z+0.10, false);
	ros::Duration(1).sleep();

	// Move Left Arm to the Home position
	home_pos("left");
	ros::Duration(1).sleep();

	// Move Left Arm Above the Object
	move_to_position(left_x, left_y, left_z, false);
	ros::Duration(1).sleep();

	// Lower the Left Arm
	move_to_position(left_x, left_y, left_z-0.10, false);
	ros::Duration(1).sleep();

	// Close the Gripper
	open_gripper("left");

	// Moving Up the Left Arm
	move_to_position(left_x, left_y, left_z+0.10, false);
	ros::Duration(1).sleep();

	// Move Left Arm to the Home position
	home_pos("left");
	ros::Duration(1).sleep();

	return 0;
}
