/** @package my_erl2
* 
* @file checkcorrect.cpp
* @brief Node to implement the check correct action 
* 
* @author Riccardo Zuppetti
* @version 1.0
* @date 09/08/2022
* @details 
*
* Subscribes to: <BR>
*    None
* 
* Publishes to: <BR>
*    None
* 
* Client: <BR>
*   /armor_interface
* 	
* Services: <BR>
*  /rosplan_interface_checkcorrect
*
* Description: <BR>
*
*  This is a service server node that if called directly ask to /armor_interface
*  service if the current consistent hypothesis is correct
*
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_erl2/ArmorInterface.h>
#include <my_erl2/CheckAction.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/checkcorrect.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <sstream>
ros::ServiceClient client;

namespace KCL_rosplan {

CheckCorrectActionInterface::CheckCorrectActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	

}

/**
* @brief : This function is the callback function of the service for server.
* @param msg : the request received from the dispatcher
* 
* @return : true
* 
* In this function is directly called the /armor_interface service to check if the current consistent hypothesis is correct.
* Return value is True if is correct and the game ended.
*/
 
bool CheckCorrectActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
	my_erl2::ArmorInterface srv;
	
	int curr_hypo;

	srv.request.mode = 1;

	if (client.call(srv))
	{
	    if(srv.response.success==false){
	       ROS_INFO("%d incorrect hypotesis",srv.response.ID);
	       ROS_INFO("Action (%s) performed: not completed!", msg->name.c_str());
	       return false;
	    }
	    
	}
	else
	{
	    ROS_ERROR("Failed to call service armor_interface");
	    return false;
	}

	ROS_INFO("%d correct hypotesis! The game is ended.",srv.response.ID);
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "check_correct_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");

client = nh.serviceClient<my_erl2::ArmorInterface>("/armor_interface");
KCL_rosplan::CheckCorrectActionInterface check(nh);

check.runActionInterface();

return 0;
}


