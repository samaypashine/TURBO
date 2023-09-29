/**
 * @file grasp_objects.cpp
 * @author Samay Pashine
 * @version 1.0
 * @date 2023-09-29
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


#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


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


/**
 * @brief States of a pose out of the way.
 * name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
 * position: [ 1.7994565963745117, -1.304173771535055, 0.46854838728904724, -2.487392250691549, -4.015228335057394, -0.6358125845538538]
 * velocity :  [-0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
 * effort: [1.1388397216796875, 2.6812055110931396, 1.275590181350708, 0.4529372453689575, 0.16012932360172272, 0.06405173242092133]
 */


class ur5Behavior
{
	public:

	ros::NodeHandle n;
	ros::Publisher pose_pub;
	ros::Publisher gripper_pub;
	ros::Publisher command_pub;
	ros::ServiceClient ik_client;
	geometry_msgs::PoseStamped current_button_pose;
	moveit::planning_interface::MoveGroupInterface *group;

	float Z_OFFSET;
	float NEW_Z_OFFSET;
	double Z_coord;
	bool g_caught_sigint;

	pcl::PointCloud<pcl::PointXYZ> target_object;
	std::string frame_id;

	ur5Behavior();
	~ur5Behavior();
	void sig_handler(int sig);
	int getch(void);
	bool close_gripper(double, double, double);
	bool open_gripper(double, double, double);
	pcl::PointCloud<pcl::PointXYZ> detectObjects();
	geometry_msgs::Pose getPoseFromObject();
	geometry_msgs::Pose getPoseFromCoord(double, double, double);
	double* inverseKinematic(geometry_msgs::Pose, double [], int);
	int trajectoryChecker(moveit_msgs::RobotTrajectory);
	void motionPlanner();
	void move_arm();
};

ur5Behavior::ur5Behavior()
{
	Z_OFFSET = 0.50;
	NEW_Z_OFFSET = 0.0;
	Z_coord = 0.704;
	g_caught_sigint = false;
    
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_arm_demo/pose", 10);
	gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 10);
	command_pub = n.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 10);
	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

	group = new moveit::planning_interface::MoveGroupInterface("right_manipulator");
	group->setPlannerId("RRTConnect");
    group->setGoalTolerance(0.01);
	group->setPlanningTime(15);

	frame_id = "camera_depth_optical_frame";
}

ur5Behavior::~ur5Behavior()
{
}

void ur5Behavior::sig_handler(int sig)
{	
	g_caught_sigint = true;
    ROS_ERROR("Caught sigint, SHUTDOWN...");
    ros::shutdown();
    exit(1);
}

int ur5Behavior::getch(void)
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

bool ur5Behavior::close_gripper(double position = 0.0, double speed = 1.0, double force = 1.0)
{
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();	
	return 0;
}

bool ur5Behavior::open_gripper(double position = 1.0, double speed = 1.0, double force = 100.0)
{	
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();
	return 0;
}

void ur5Behavior::move_arm()
{
	ROS_INFO("Moving the Arm.");
	group->move();
	ros::spinOnce();
}

pcl::PointCloud<pcl::PointXYZ> ur5Behavior::detectObjects()
{
	pcl::PointCloud<pcl::PointXYZ> object;
	ros::ServiceClient client = n.serviceClient<turbo_robot_vision::TabletopPerception>("/turbo_object_detector/detect");
	turbo_robot_vision::TabletopPerception srv;
	
	ROS_INFO("Detecting the Objects");
	if(client.call(srv))
	{	
		// Shut Down if the cannot find the plane.
		if(srv.response.is_plane_found == false)
		{
			ROS_ERROR("No object Found. Exitting the Code.");
			ros::shutdown();
		}
		
		int num_objects = srv.response.cloud_clusters.size();
		std::vector<pcl::PointCloud<pcl::PointXYZ>> detected_objects;
		ROS_INFO("Number of Objects Found : %i", num_objects);

		// Convert object to PCL format		
		for (int i = 0; i < num_objects; i++)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud_i;
			pcl::fromROSMsg(srv.response.cloud_clusters[i], cloud_i);
			detected_objects.push_back(cloud_i);
		}

		// Find the largest object out of all.
		int object_index = 0;
		int max = detected_objects[0].points.size();
		for (int i = 0; i < detected_objects.size(); i++)
		{
			int num_points = detected_objects[i].points.size();
			if (num_points > max)
			{
				max = num_points;
				object_index = i;
			}
		}		
		object = detected_objects[object_index];
		target_object = object;
		frame_id = object.header.frame_id;
		ROS_INFO_STREAM(frame_id);
	}

	ros::spinOnce();
	return object;
}

geometry_msgs::Pose ur5Behavior::getPoseFromObject()
{
	geometry_msgs::Pose pose_i;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(target_object, centroid);

	pose_i.position.x = centroid(0);
	pose_i.position.y = centroid(1);
	pose_i.position.z = Z_coord;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.50, 0, 0);
	return pose_i;
}

geometry_msgs::Pose ur5Behavior::getPoseFromCoord(double x, double y, double z)
{
	geometry_msgs::Pose pose_i;
	pose_i.position.x = x;
	pose_i.position.y = y;
	pose_i.position.z = z;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	return pose_i;
}

int ur5Behavior::trajectoryChecker(moveit_msgs::RobotTrajectory trajectoryPlan)
{
	ROS_INFO("Checking the Trajectory");
	std::vector<trajectory_msgs::JointTrajectoryPoint> trajectoryPoints;
	trajectoryPoints = trajectoryPlan.joint_trajectory.points;
	std::vector<int>::size_type vectorSize = trajectoryPoints.size();

	for (int i = 0; i < vectorSize; i++)
	{
		if (trajectoryPoints[i].positions[0] < -1.74 || trajectoryPoints[i].positions[0] > 3.14)
		{	
			ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 0");
			std::cout<<"Point : "<<i;
			getch();
			return 0;
		}
		
		if (trajectoryPoints[i].positions[1] < -2.61 || trajectoryPoints[i].positions[1] > 3.14)
		{	
			ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 1");
			std::cout<<"Point : "<<i;
			getch();
			return 0;
		}
		
		if (trajectoryPoints[i].positions[2] < 0.50 || trajectoryPoints[i].positions[2] > 3.14)
		{	
			ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 2");
			std::cout<<"Point : "<<i;
			getch();
			return 0;
		}

		// if (trajectoryPoints[i].positions[3] > 5.00000 || trajectoryPoints[i].positions[3] < 3.50000)
		// {	
		// 	ROS_INFO_STREAM(trajectoryPlan);
		// 	ROS_WARN("Trajectory not Feasible due to position 3");
		// 	std::cout<<"Point : "<<i;
		// 	getch();
		// 	return 0;
		// }

		// if (trajectoryPoints[i].positions[4] > -3.80000 || trajectoryPoints[i].positions[4] < -4.60000)
		// {	
		// 	ROS_INFO_STREAM(trajectoryPlan);
		// 	ROS_WARN("Trajectory not Feasible due to position 4");
		// 	std::cout<<"Point : "<<i;
		// 	getch();
		// 	return 0;
		// }

		// if (trajectoryPoints[i].positions[5] > 1.370000 || trajectoryPoints[i].positions[5] < -6.67000)
		// {	
		// 	ROS_INFO_STREAM(trajectoryPlan);
		// 	ROS_WARN("Trajectory not Feasible due to position 5");
		// 	std::cout<<"Point : "<<i;
		// 	getch();
		// 	return 0;
		// }
	}
	ros::spinOnce();
	return 1;
}

double* ur5Behavior::inverseKinematic(geometry_msgs::Pose pose_i, double joint[6], int planFlag = 0)
{
	geometry_msgs::PoseStamped stampedPose, stampOut;
	tf::TransformListener listener;
	moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;

	stampedPose.header.frame_id = frame_id;
	stampedPose.header.stamp = ros::Time(0);
	stampedPose.pose = pose_i;
 
	listener.waitForTransform(stampedPose.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
	listener.transformPose("world", stampedPose, stampOut);

	stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);
	stampOut.pose.position.z += Z_OFFSET;

	ROS_INFO("Publishing the Pose");
	pose_pub.publish(stampOut);

	if (planFlag == 1)
	{
		group->setPoseReferenceFrame(stampOut.header.frame_id);
		group->setPoseTarget(stampOut);
		group->setStartState(*group->getCurrentState());
	}
    
	ik_request.ik_request.group_name = "right_manipulator";
    ik_request.ik_request.pose_stamped = stampOut;
    ik_client.call(ik_request, ik_response);

	joint[0] = ik_response.solution.joint_state.position[0];
	joint[1] = ik_response.solution.joint_state.position[1];
	joint[2] = ik_response.solution.joint_state.position[2];
	joint[3] = ik_response.solution.joint_state.position[3];
	joint[4] = ik_response.solution.joint_state.position[4];
	joint[5] = ik_response.solution.joint_state.position[5];
	
	ros::spinOnce();
	return joint;
}

void ur5Behavior::motionPlanner()
{
	double temp_joint[6];
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	geometry_msgs::Pose pose_i = getPoseFromObject();	
	double* joint_angles = inverseKinematic(pose_i, temp_joint, 1);

	int choice = 0;
	int numOfTries = 10;
	while(choice != 1)
	{
		ROS_INFO("Planning the Motion");
		moveit::planning_interface::MoveItErrorCode planStatus = group->plan(my_plan);
		ROS_INFO("Plan Status : %s", planStatus ? "Success" : "Failed");
		
		if (planStatus)
		{
			moveit_msgs::RobotTrajectory trajectoryPlan = my_plan.trajectory_;
			choice = trajectoryChecker(trajectoryPlan);

			if (choice == 0)
			{
				ROS_WARN("Trajectory Status : NOT GOOD");
				if (numOfTries > 0)
					numOfTries -= 1;
			}
			else
			{
				ROS_INFO("Trajectory Status : Good");
			}
		}

		if (numOfTries == 0 && choice == 0)
		{
			ROS_ERROR("No Feasible Plan possible for the position.");
			ROS_INFO("Exitting the Code.");
			ros::shutdown();
		}
	}
	ROS_INFO("Executing Plan.");
	group->execute(my_plan);
	ros::spinOnce();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_arm");
	ros::AsyncSpinner spinner(0);
	spinner.start();

	ur5Behavior Obj;
	pcl::PointCloud<pcl::PointXYZ> demo_pose = Obj.detectObjects();
	Obj.motionPlanner();
	
	spinner.stop();
	return 0;
}
