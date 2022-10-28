/**
 * This file defines the CheckConsistencyActionInterface class.
 * CheckConsistencyActionInterface is used to check if there is a new consistent hypotesis
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class CheckConsistencyActionInterface: public RPActionInterface
	{

	private:
               //ros::ServiceClient client;
	public:

		/* constructor */
		CheckConsistencyActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
