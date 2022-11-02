/** @package my_erl2
* 
* @file checkconsistency.cpp
* @brief Node to implement the check consistency action 
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
*  /armor_interface
* 	
* Services: <BR>
*  /rosplan_interface_checkconsistency
*
* Description: <BR>
*
* This is a service server node that if called directly ask to /armor_interface
* service if there is a new consistent hypothesis to check.
*
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_erl2/ArmorInterface.h>
#include <my_erl2/CheckAction.h>
#include "my_erl2/checkconsistency.h"
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include <motion_plan/PlanningAction.h>

ros::ServiceClient client;

namespace KCL_rosplan {

CheckConsistencyActionInterface::CheckConsistencyActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	

}

/**
* @brief : This function is the callback function of the service for server.
* @param msg : the request received from the dispatcher
* 
* @return : true 
* 
* In this function is directly called the /armor_interface service to check if there is a new consistent hypothesis.
* Return value is True if there is.
*/
 
bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
	my_erl2::ArmorInterface srv;
	srv.request.mode = 2;

	if (client.call(srv))
	{
	    if(srv.response.success==false){
	       ROS_INFO("no new consistent hypotesis to check");
	       ROS_INFO("Action (%s) performed: not completed!", msg->name.c_str());
	       return false;
	    }
	    
	}
	else
	{
	    ROS_ERROR("Failed to call service armor_interface");
	    return false;
	}

	ROS_INFO("%d new consistent hypotesis to check!",srv.response.ID);
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "check_concistency_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");

client = nh.serviceClient<my_erl2::ArmorInterface>("/armor_interface");
KCL_rosplan::CheckConsistencyActionInterface my_aci(nh);

my_aci.runActionInterface();

return 0;
}


