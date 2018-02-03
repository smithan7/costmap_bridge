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

	int test_environment_number = 0;
	int agent_index = 0;
	int jetson = 0;
	int parameter_seed = 0;

	ros::param::get("test_environment_number", test_environment_number);
	ros::param::get("agent_index", agent_index);
	ros::param::get("jetson", jetson);
	ros::param::get("/dmcts/parameter_seed", parameter_seed);
	

	ROS_INFO("Costmap_Bridge::initializing costmap");
	ROS_INFO("   test_environment_number %i", test_environment_number);
	ROS_INFO("   agent_index %i", agent_index);
	ROS_INFO("   jetson %i", jetson);
	ROS_INFO("   parameter_seed %i", parameter_seed);
	
	Costmap *costmap = new Costmap(nHandle, test_environment_number, agent_index, jetson, parameter_seed);

	// return the control to ROS
	ros::spin();

	return 0;
}
