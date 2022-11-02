/** @package my_erl2
* 
* @file grippermotion.cpp
* @brief Node to implement the motion of the gripper
*
* @author Riccardo Zuppetti
* @version 1.0
* @date 09/08/2022
* @details 
*
*
* Subscribes to: <BR>
*    None
* 
* Publishes to: <BR>
*    None
* 
* Client: <BR>
*   /moveit_interface
* 	
* Services: <BR>
*  /rosplan_interface_grippermotion
*
* Description: <BR>
*
*  This is a service node used to control the robot in order to reach a specific location in the environment
*
*/

#include <unistd.h>
#include <ros/ros.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/grippermotion.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "geometry_msgs/Pose.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include <string>
#include <iostream>

namespace KCL_rosplan {
GripperMotionActionInterface::GripperMotionActionInterface(ros::NodeHandle &nh) {
// here the initialization

}

/**
* @brief : This function is the callback function of the service for server.
* @param msg : the request received from the dispatcher
* 
* @return : true A boolean value
* 
* In this function The moveit ad-hoc generated package is used in order to reach a certain point in the environment.
* Return value is True if the point is reached.
*/

bool GripperMotionActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
         //moveit initialization
	 robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	 const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	 moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	 kinematic_state->setToDefaultValues();
	 const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	 moveit::planning_interface::MoveGroupInterface group("arm");
	 const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

         int down;
          //get the actual_location of the robot
          std::string actual_location;
          ros::param::get("/actual_location", actual_location);
          //understand if in that location the position to reach is 0.75 or 1.25
          ros::param::get(actual_location, down);
          
          geometry_msgs::Pose pose1;
	  std::vector<double> joint_values;
	  double timeout = 0.1;	
          bool found_ik;
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          //set the goal pose
	  pose1.orientation.w = 0.70;
	  pose1.orientation.x = -0.00;
	  pose1.orientation.y = 0.00;
	  pose1.orientation.z = -0.71;
	  pose1.position.x =  0.5;
	  pose1.position.y =  0;
	  //query about location
	  if(down==1)	pose1.position.z =  0.75;
	  else pose1.position.z =  1.25;
	  //set the current state as the start state
	  group.setStartStateToCurrentState();
	  group.setApproximateJointValueTarget(pose1, "arm_link_04");
          //found the inverse kinematic in order to reach the goal
	  found_ik = kinematic_state->setFromIK(joint_model_group, pose1, timeout);

	  // Now, we can print out the IK solution (if found):
	  if (found_ik)
	  {
	    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	    for (std::size_t i = 0; i < joint_names.size(); ++i)
	    {
	      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	    }
	  }
	  else
	  {
	    ROS_INFO("Did not find IK solution");
	  }
	  
	  
	  group.setJointValueTarget(joint_values);
	  group.setStartStateToCurrentState();
	  group.setGoalOrientationTolerance(0.01);
	  group.setGoalPositionTolerance(0.01);

	  // Plan and execute
	  
	  group.plan(my_plan); 
	  group.execute(my_plan);
	  
	  std::cout << "Position 1 -> IK + setJointValue" << std::endl;

          
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "gripper_motion", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::GripperMotionActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}

