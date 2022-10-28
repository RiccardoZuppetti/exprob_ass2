/**
 * This file defines the MoveActionInterface class.
 * MoveActionInterface is used to control robot navigationbetween locations
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class MoveActionInterface: public RPActionInterface
	{

	private:

	public:

		/* constructor */
		MoveActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

