#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Costmap.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "Costmap");
	ros::NodeHandle nHandle("~");
	int test_environment_number = 3;
	int agent_index = 0;
	ROS_INFO("Costmap_Bridge::initializing costmap");
	Costmap *costmap = new Costmap(nHandle, test_environment_number, agent_index);

	// return the control to ROS
	ros::spin();

	return 0;
}
