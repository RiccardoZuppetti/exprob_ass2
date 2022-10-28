/** @package my_erl2
* 
* @file save_erloracle.cpp
* @brief Node to save erloracle
* @author Riccardo Zuppetti
* @version 1.0
* @date 09/08/2022
*
*
*
* @details 
*
* Subscribes to: <BR>
*    /oracle_hint
* 
* Publishes to: <BR>
*    None
* 
* Client: <BR>
*    None
*   
* Services: <BR>
*   None
*
* Action Client:
*   None
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
#include <motion_plan/PlanningAction.h>
#include "my_erl2/ErlOracle.h"

void Callback(const my_erl2::ErlOracle::ConstPtr& msg)
{
  std::cout<<"NEW"<<std::endl;
  std::cout<<msg->ID<<std::endl;
  std::cout<<msg->key<<std::endl;
  std::cout<<msg->value<<std::endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "save_erloracle", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");	
	ros::Subscriber sub = nh.subscribe("/oracle_hint", 1000, Callback);       
        ros::Rate loop_rate(10);
        while (ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
        }
	return 0;
}
