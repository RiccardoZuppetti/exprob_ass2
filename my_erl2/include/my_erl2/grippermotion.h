/**
 * This file defines the GripperMotionActionInterface class.
 * GripperMotionActionInterface is used to simulate control the robotic arm
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class GripperMotionActionInterface: public RPActionInterface
	{

	private:
	       

	public:

		/* constructor */
		GripperMotionActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

