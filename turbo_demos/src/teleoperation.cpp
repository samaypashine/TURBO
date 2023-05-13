/**
 * @file teleoperation.cpp
 * @author Samay Pashine
 * @brief This is a demo file to move both of the arm one at a time, using URScript command.
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
#include <moveit_msgs/GetPositionFK.h>

#include <robotiq_85_msgs/GripperCmd.h>


ros::ServiceClient ik_client;
ros::ServiceClient fk_client;

ros::Publisher left_command_pub;
ros::Publisher right_command_pub;

moveit::planning_interface::MoveGroupInterface *left_arm_group;
moveit::planning_interface::MoveGroupInterface *right_arm_group;

float Z_OFFSET = 0.25;

/**
 * @brief Function to block the execution temporarily.
 * 
 */
int pressEnter()
{
	struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}



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



geometry_msgs::Pose getPoseFromCoord(double x, double y, double z)
{
	geometry_msgs::Pose pose_i;
	pose_i.position.x = x;
	pose_i.position.y = y;
	pose_i.position.z = z;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	return pose_i;
}



std_msgs::String URScriptCommand(double joint[], double a = 0.20, double v = 0.20, double t = 0, double r = 0)
{
	std_msgs::String command; 
	command.data = "movej([";

	command.data += std::to_string(joint[0]);
	command.data += ",";

	command.data += std::to_string(joint[1]);
	command.data += ",";

	command.data += std::to_string(joint[2]);
	command.data += ",";

	command.data += std::to_string(joint[3]);
	command.data += ",";

	command.data += std::to_string(joint[4]);
	command.data += ",";

	command.data += std::to_string(joint[5]);
	command.data += "],a=";
	
	command.data += std::to_string(a);
	command.data += ",v=";

	command.data += std::to_string(v);
	command.data += ",t=";

	command.data += std::to_string(t);
	command.data += ",r=";
	
	command.data += std::to_string(r);
	command.data += ")\n";

	return command;
}


double* inverseKinematic(geometry_msgs::Pose pose_i, double joint[6], bool flag)
{
	geometry_msgs::PoseStamped stampedPose;
	moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;

	ROS_INFO("PHASE 2 - 1");

	stampedPose.header.frame_id = "world";
	stampedPose.header.stamp = ros::Time(0);
	stampedPose.pose = pose_i;
	stampedPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);

	ROS_INFO("PHASE 2 - 2");
	ROS_INFO_STREAM(stampedPose);

	if (flag)
	{
		right_arm_group->setPoseTarget(stampedPose);
		right_arm_group->setStartState(*right_arm_group->getCurrentState());
		ik_request.ik_request.group_name = "right_manipulator";
	}
	else
	{
		left_arm_group->setPoseTarget(stampedPose);
		left_arm_group->setStartState(*left_arm_group->getCurrentState());
		ik_request.ik_request.group_name = "left_manipulator";
	}

	ROS_INFO("PHASE 2 - 3");

    ik_request.ik_request.pose_stamped = stampedPose;
    ik_client.call(ik_request, ik_response);

	ROS_INFO("PHASE 2 - 4");
	ROS_INFO_STREAM(ik_response.solution.joint_state);
	

	if (flag)
	{
		joint[0] = ik_response.solution.joint_state.position[12];
		joint[1] = ik_response.solution.joint_state.position[13];
		joint[2] = ik_response.solution.joint_state.position[14];
		joint[3] = ik_response.solution.joint_state.position[15];
		joint[4] = ik_response.solution.joint_state.position[16];
		joint[5] = ik_response.solution.joint_state.position[17];
	}
	else
	{
		joint[0] = ik_response.solution.joint_state.position[0];
		joint[1] = ik_response.solution.joint_state.position[1];
		joint[2] = ik_response.solution.joint_state.position[2];
		joint[3] = ik_response.solution.joint_state.position[3];
		joint[4] = ik_response.solution.joint_state.position[4];
		joint[5] = ik_response.solution.joint_state.position[5];
	}

	ROS_INFO("PHASE 2 - 5");
	return joint;
}



void cartesianControl()
{
	double left_x = 0.875933;
	double left_y = 0.0130693;
	double left_z = 1.04481;

	double left_Rx = 0;
	double left_Ry = 0;
	double left_Rz = 0;

	double left_a = 0.25;
	double left_v = 0.25;
	double left_t_ = 0;
	double left_r = 0;

	double right_x = -0.0344373;
	double right_y = 0.0549475;
	double right_z = 1.01966;

	double right_Rx = 0;
	double right_Ry = 0;
	double right_Rz = 0;

	double right_a = 0.25;
	double right_v = 0.25;
	double right_t_ = 0;
	double right_r = 0;

	int trigger = 99;

	ROS_INFO("PHASE 1");

	// Press '0' (Zero) to exit the teleoperation loop
	while (trigger != 48)
	{
		std::cout<<"Controller : ";
		trigger = pressEnter();
		std::cout<<trigger<<"\n";

		switch (trigger)
		{
			case 105: // i - key
						left_z += 0.005;
						break;

			case 107: // k - key
						left_z -= 0.005;
						break;

			case 108: // l - key
						left_x += 0.005;
						break;

			case 106: // j - key
						left_x -= 0.005;
						break;

			case 111: // o - key
						left_y += 0.005;
						break;

			case 117: // u - key
						left_y -= 0.005;
						break;
			

			case 119: // w - key
						right_z += 0.005;
						break;

			case 115: // s - key
						right_z -= 0.005;
						break;

			case 100: // d - key
						right_x += 0.005;
						break;

			case 97: // a - key
						right_x -= 0.005;
						break;

			case 101: // e - key
						right_y += 0.005;
						break;

			case 113: // q - key
						right_y -= 0.005;
						break;
		}

		double temp_joints[6];
		double* joint_angles;
		std_msgs::String command;

		ROS_INFO("PHASE 2");

		geometry_msgs::Pose right_pose_i = getPoseFromCoord(right_x, right_y, right_z);	
		joint_angles = inverseKinematic(right_pose_i, temp_joints, true);
		command = URScriptCommand(joint_angles, right_a, right_v, right_t_, right_r); 
		ROS_INFO_STREAM(command);
		right_command_pub.publish(command);
		
		ROS_INFO("PHASE 3");

		geometry_msgs::Pose left_pose_i = getPoseFromCoord(left_x, left_y, left_z);	
		joint_angles = inverseKinematic(left_pose_i, temp_joints, false);
		command = URScriptCommand(joint_angles, left_a, left_v, left_t_, left_r); 
		ROS_INFO_STREAM(command);
		left_command_pub.publish(command);
	
		ROS_INFO("PHASE 4");
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turbo_demo_teleoperation");

    ros::NodeHandle n;

	ros::AsyncSpinner spinner(0);
	spinner.start();

    ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	fk_client = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

	moveit_msgs::GetPositionFK::Request fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;

    left_command_pub = n.advertise<std_msgs::String>("/left/ur_hardware_interface/script_command", 10);
    right_command_pub = n.advertise<std_msgs::String>("/right/ur_hardware_interface/script_command", 10);

    left_arm_group = new moveit::planning_interface::MoveGroupInterface("left_manipulator");
    right_arm_group = new moveit::planning_interface::MoveGroupInterface("right_manipulator");

    left_arm_group->setPlannerId("RRTConfigDefault");
    left_arm_group->setGoalTolerance(0.01);
    left_arm_group->setPlanningTime(10);
	left_arm_group->setPoseReferenceFrame("world");

    right_arm_group->setPlannerId("RRTConfigDefault");
    right_arm_group->setGoalTolerance(0.01);
    right_arm_group->setPlanningTime(10);
	right_arm_group->setPoseReferenceFrame("world");

    ROS_INFO("Moving Left Arm to Home Position.");
    home_pos("left");

    ROS_INFO("Moving Right Arm to Home Position.");
    home_pos("right");
    pressEnter();

	cartesianControl();
    
	// fk_request.fk_link_names.push_back("left_ee_link");
    // fk_request.header.frame_id = "world";
    // fk_request.robot_state.joint_state.position = {-0.02146755, -2.02021861, -1.583014, -0.97930424, -2.49791523, 0.05602507};

    // fk_client.call(fk_request, fk_response);
	// const geometry_msgs::PoseStamped& end_effector_pose = fk_response.pose_stamped[0];
	// const geometry_msgs::Point& end_effector_position = end_effector_pose.pose.position;

	// ROS_INFO_STREAM(end_effector_pose);
	// ROS_INFO_STREAM(end_effector_position);

    spinner.stop();
    return 0;
}