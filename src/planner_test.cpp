#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
#include "moveit_utils/MicoController.h"
#include <segbot_arm_manipulation/arm_utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ctime>


using namespace std;


clock_t begin;
clock_t end;

geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped pose_1;
geometry_msgs::PoseStamped pose_2;
geometry_msgs::PoseStamped pose_3;
geometry_msgs::PoseStamped pose_4;
ros::ServiceClient controller_client;

bool service_cb(geometry_msgs::PoseStamped p_target){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setPlanningTime(10.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = p_target.pose.orientation;
    goal.pose.position = p_target.pose.position;
    //publish target pose
    //pose_pub_target.publish(p_target);
    //ROS_INFO_STREAM(end_pose);
    
    group.setStartState(*group.getCurrentState());
    group.setPoseTarget(p_target);

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    
    begin = clock();
    bool success = group.plan(my_plan);
    double duration; 
    if(success){
		end = clock();
		duration = double(end - begin)/(CLOCKS_PER_SEC/1000); // MILLISECONDS
		ROS_INFO("planning successful\n");
	}
	else {
		end = clock();
		duration = double(end - begin)/(CLOCKS_PER_SEC/1000); // MILLISECONDS
		ROS_INFO("not successful :( \n");
	}
	
	ROS_INFO("DURATION FOR PLANNING: ");
	ROS_INFO_STREAM(duration);
			
    //call service
    // ROS_INFO("Printing Trajectory \n");
    // ROS_INFO_STREAM(my_plan.trajectory_);
    moveit_utils::MicoController srv;
    srv_controller.request.trajectory = my_plan.trajectory_;

	
	ROS_INFO("CALLING CONTROLLER CLIENT.");
	
    if(controller_client.call(srv_controller)){
       ROS_INFO("Service call sent. Prepare for movement.");
       //res.completed = srv_controller.response.done;
    }
    else {
      ROS_INFO("Service call failed. Is the service running?");
      //res.completed = false;
    }
  
    ros::spinOnce();
    return true;
}

//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void pressEnter(){
  std::cout << "Press the ENTER key to continue";
  while (std::cin.get() != '\n')
    std::cout << "Please press ENTER\n";
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  controller_client = nh.serviceClient<moveit_utils::MicoController>("mico_controller");
  signal(SIGINT, sig_handler);
  pressEnter();
  ROS_INFO("Planner Testing Starting...");
  ROS_INFO("Moving Arm to starting position");
  start_pose.header.frame_id = "mico_link_base";
  start_pose.pose.position.x = 0.181252196431; 
  start_pose.pose.position.y = 0.50459575653; 
  start_pose.pose.position.z = 0.192858144641; 
  start_pose.pose.orientation.x = 0.196969260996;
  start_pose.pose.orientation.y = 0.643877379238;
  start_pose.pose.orientation.z = 0.708712907086;
  start_pose.pose.orientation.w = 0.21059688045;
  //pose_1.orientation.x = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
  ROS_INFO("Moving to Start Pose");
  ROS_INFO_STREAM(start_pose);
  service_cb(start_pose);
  
  pose_1.header.frame_id = "mico_link_base";
  pose_1.pose.position.x = 0.321145832539; 
  pose_1.pose.position.y = 0.376617610455; 
  pose_1.pose.position.z = 0.362925946712; 
  pose_1.pose.orientation.x = 0.3316365428476;
  pose_1.pose.orientation.y = 0.586248103624;
  pose_1.pose.orientation.z = 0.642069363965;
  pose_1.pose.orientation.w = 0.366165667839;
  ROS_INFO("Now Moving to Pose 1");
  ROS_INFO_STREAM(pose_1);
  service_cb(pose_1);
  
  //segbot_arm_manipulation::moveToPoseMoveIt(nh,pose_1);
  ros::spin();

  return 0;
}
