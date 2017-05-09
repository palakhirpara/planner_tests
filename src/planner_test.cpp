  #include <signal.h>
  #include "ros/ros.h"
  #include "geometry_msgs/PoseStamped.h"
  #include <iostream>
  #include <fstream>
  #include <string>
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
  #include <sys/time.h> 


  using namespace std;
  struct timeval begin, end;
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped pose_1;
  geometry_msgs::PoseStamped pose_2;
  geometry_msgs::PoseStamped pose_3;
  geometry_msgs::PoseStamped pose_4;
  ros::ServiceClient controller_client;
  double total_planning_time = 0.0;
  double total_trajectory_time = 0.0;
  double duration_expected = 0.0;
  std::vector<trajectory_msgs::JointTrajectoryPoint> t_points;

  ofstream outfile;
  ofstream outfilecsv;


  void service_cb(geometry_msgs::PoseStamped p_target, int pose_number, string pn){
      ROS_INFO("Inside Service CB");
      
      moveit_utils::MicoController srv_controller;
      moveit::planning_interface::MoveGroup group("arm");
      // See ompl_planning.yaml for a complete list
      group.setPlannerId(pn);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      group.setPlanningTime(10.0); //5 second maximum for collision computation
      moveit::planning_interface::MoveGroup::Plan my_plan;
      
      geometry_msgs::PoseStamped goal;
      goal.pose.orientation = p_target.pose.orientation;
      goal.pose.position = p_target.pose.position;
      group.setStartState(*group.getCurrentState());
      group.setPoseTarget(p_target);
      gettimeofday(&begin, NULL);
      bool success = group.plan(my_plan);
      double duration; 
      if(success){
  		  gettimeofday(&end, NULL);
  		  //ROS_INFO("planning successful\n");
  	  }
  	  else {
  		  gettimeofday(&end, NULL);
  		  ROS_INFO("not successful :( \n");
  	  }
  	
    	// Calculate the planning time
    	duration = (end.tv_sec - begin.tv_sec) * 1000.0;      // sec to ms
    	duration += (end.tv_usec - begin.tv_usec) / 1000.0;   // us to ms
    	total_planning_time += duration;
    	outfile << "Planning Time for Pose " << pose_number << ": " << duration << " ms" << endl;
    	outfilecsv << pose_number << "," << duration << ",";

      // Calculate the trajectory time
      t_points = my_plan.trajectory_.joint_trajectory.points;
      int size = t_points.size();
      
      if(success && size > 1){
    	
        duration_expected = t_points.at(t_points.size()-1).time_from_start.toSec();
        total_trajectory_time += duration_expected;
        outfile << "Trajectory Time for Pose " << pose_number << ": " << duration_expected << " seconds" << endl;
        outfilecsv << duration_expected << endl;
        //ROS_INFO_STREAM(my_plan.trajectory_);
      }
      else{
    		outfile << "Not SUccessful " << endl;
    		outfilecsv << "Not SUCCESSFUL" << endl;
    	}
      
      return;
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


  void move_to_pose_and_measure(double px, double py, 
  							  double pz, double ox, double oy, double oz, double ow, int pose_number, string pn){
    geometry_msgs::PoseStamped pose;
    double duration;
    pose.header.frame_id = "mico_link_base";
    pose.pose.position.x = px; 
    pose.pose.position.y = py; 
    pose.pose.position.z = pz; 
    pose.pose.orientation.x = ox;
    pose.pose.orientation.y = oy;
    pose.pose.orientation.z = oz;
    pose.pose.orientation.w = ow;
    service_cb(pose, pose_number, pn);

  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "planner_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    double duration; 
    string planners[11] = {"SBLkConfigDefault", "ESTkConfigDefault", "LBKPIECEkConfigDefault" 
  	  ,"BKPIECEkConfigDefault","KPIECEkConfigDefault","RRTkConfigDefault"
  	  ,"RRTConnectkConfigDefault","RRTstarkConfigDefault","TRRTkConfigDefault"
        ,"PRMkConfigDefault","PRMstarkConfigDefault"}; 
    
    outfile.open("/home/users/palak96/catkin_ws/src/planner_tests/planning_data.txt" , ios::out | ios::app );
    outfilecsv.open("/home/users/palak96/catkin_ws/src/planner_tests/planning_data.csv" , ios::out | ios::app ); 
      
    segbot_arm_manipulation::closeHand();
    signal(SIGINT, sig_handler);
    pressEnter();
    ROS_INFO("Planner Testing Starting...");
    
    // change 2 to 11;
    for(int i = 0; i < 11; i++){
    cout << "Planner Name: " << planners[i] << endl << endl;
    outfile << "Planner Name: " << planners[i] << endl << endl;
    outfilecsv << planners[i] << endl;
    // pose 0
    move_to_pose_and_measure(0.181252196431, 0.50459575653, 
  							  0.192858144641, 0.196969260996, 0.643877379238, 
  							  0.708712907086, 0.2105968804, 0, planners[i]);
    // pose 1					
    move_to_pose_and_measure(0.321145832539, 0.376617610455, 
  							  0.362925946712, 0.3316365428476, 0.58624810362, 
  							  0.6420693639656, 0.366165667839, 1, planners[i]);
    // pose 2	
    move_to_pose_and_measure(0.46166241169, 0.0376703366637, 
  							  0.199446335435, 0.574395015985, 0.3408209072 , 
  							  0.361313489507, 0.65066430448, 2, planners[i]);
    // pose 3		
    move_to_pose_and_measure(0.260384321213, -0.187947839499, 
  							  0.308621138334, 0.616884295636, 0.511692874143, 
  							  0.368341805462, 0.4711140867129, 3, planners[i]);
    // pose 4	
    move_to_pose_and_measure(0.114481061697,  -0.455266356468, 
  							  0.300507098436, 0.682321049804, 0.175997478152, 
  							  0.176532227353, 0.687240311233, 4, planners[i]);
  							
    // pose 5	
    move_to_pose_and_measure(0.511899292469,  0.0834245383739, 
  							  0.00473425397649, 0.310907854703, 0.742598419966, 
  							  0.513918963892, 0.296262031149, 5, planners[i]);

    // print to console						  
    cout << endl << "++++++++++ Total Planning Time: " <<
  		total_planning_time << " ms or " << total_planning_time/1000 << " sec" << endl;
    cout << endl << "++++++++++ Total Trajectory Time: " << 
          total_trajectory_time << " ms or " << total_trajectory_time/1000 << " sec" << endl;
    
    // write results to a file
    outfile << "++++++++++ Total Planning Time: " <<
  		total_planning_time << " ms or " << total_planning_time/1000 << " sec" << endl;  
    outfile << "++++++++++ Total Trajectory Time: " << 
          total_trajectory_time  << " sec" << endl << endl;

    outfilecsv << "Total" << "," << total_planning_time << "," << total_trajectory_time << endl;
    total_planning_time = 0.0;
    total_trajectory_time = 0.0;
    
    }
    outfile << "End of Experiment " << endl << endl;
    outfilecsv << "End of Experiment" << endl << endl;
    outfilecsv.close();
    outfile.close();
    return 0;
  }
