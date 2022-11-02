/** @package my_erl2
* 
* @file move.cpp
* @brief Here the robot motion is implemented
*
* @author Riccardo Zuppetti
* @version 1.0
* @date 09/08/2022
* @details 
*
*
* Subscribes to: <BR>
*    /odom
* 
* Publishes to: <BR>
*    /cmd_vel
* 
* Client: <BR>
*    None
* 	
* Services: <BR>
*    /rosplan_interface_move
*
* Action Client:
*    /reaching_goal
*
*/

#include <unistd.h>
#include <ros/ros.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/move.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

int actual_x;
int actual_y;
ros::Publisher pub;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    actual_x=msg->pose.pose.position.x;
    actual_y=msg->pose.pose.position.y;    
}
int distance(int x,int y){
      return sqrt(x^2 + y^2);
}

namespace KCL_rosplan {
	
MoveActionInterface::MoveActionInterface(ros::NodeHandle &nh) {
// here the initialization
}



bool MoveActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
                
                
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		motion_plan::PlanningGoal goal;
		ac.waitForServer();
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		ac.sendGoal(goal);
		ac.waitForResult();
		ac.waitForServer();
		if(msg->parameters[2].value == "wp1"){
		goal.target_pose.pose.position.x = 2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp2"){
		goal.target_pose.pose.position.x = -2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp3"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.6;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else {
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.6;
		goal.target_pose.pose.orientation.w = 0.0;
		}
                ros::param::set("/actual_location", msg->parameters[2].value);
		ac.sendGoal(goal);
		ac.waitForResult();
                
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		
		return true;

}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "rosplan_interface_move", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
ros::Subscriber sub=nh.subscribe("/odom",1000,positionCallback);
pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
KCL_rosplan::MoveActionInterface move(nh);
move.runActionInterface();
return 0;
}

