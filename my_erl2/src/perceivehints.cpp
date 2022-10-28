/** @package my_erl2
* 
* @file perceivehints.cpp
* @brief Node to implement the check correct action 
* @author Riccardo Zuppetti
* @version 1.0
* @date 09/08/2022
*
*
*
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
*  /rosplan_interface_perceivehints
*
* Description: <BR>
*
*  This is a service node used to call the /armor_interface service in order to perceive an hint.
*
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "my_erl2/ArmorInterface.h"
#include <my_erl2/CheckAction.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/perceivehints.h"
#include <motion_plan/PlanningAction.h>

ros::ServiceClient client;

namespace KCL_rosplan {

PerceiveHintsActionInterface::PerceiveHintsActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	

}
/**
* @brief This function is the callback function of the service for server.
* @param msg : the request received from the dispatcher
* 
* @return : true 
* 
* In this function is directly called the /armor_interface service to perceive a new hint
* Return value is True if an hint is perceived.
*/
bool PerceiveHintsActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
	my_erl2::ArmorInterface srv;
	srv.request.mode = 3;
        std::cout<<srv.request<<std::endl;
	client.call(srv);
	std::cout<<srv.response<<std::endl;
	if(srv.response.success==false){
	       return false;
	}
	    

	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "perceive_hint_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");
client = nh.serviceClient<my_erl2::ArmorInterface>("/armor_interface");

KCL_rosplan::PerceiveHintsActionInterface my_aci(nh);

my_aci.runActionInterface();

return 0;
}


