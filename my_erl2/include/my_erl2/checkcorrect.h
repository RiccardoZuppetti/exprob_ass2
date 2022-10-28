/**
 * This file defines the CheckCorrectActionInterface class.
 * CheckCorrectActionInterface is used to check if the current consisten hypothesis is correct
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class CheckCorrectActionInterface: public RPActionInterface
	{

	private:
                //ros::ServiceClient client;
             
	public:

		/* constructor */
		CheckCorrectActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
