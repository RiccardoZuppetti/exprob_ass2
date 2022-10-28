/**
 * This file defines the PerceiveHintsActionInterface class.
 *PerceiveHintsActionInterface is used to perceive a new hint
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class PerceiveHintsActionInterface: public RPActionInterface
	{

	private:
                //ros::ServiceClient client;
	public:

		/* constructor */
		PerceiveHintsActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

