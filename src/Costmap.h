/*
 * Costmap.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Costmap_H_
#define SRC_Costmap_H_

// useful ROS libs
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> // action server
#include <visualization_msgs/Marker.h> // for making marks in rviz
#include <visualization_msgs/MarkerArray.h> // arrays of markers
#include <costmap_2d/costmap_2d.h>
#include "geometry_msgs/PoseStamped.h"

// talk to the DJI Bridge
#include <custom_messages/DJI_Bridge_Status_MSG.h>
#include <custom_messages/DJI_Bridge_Travel_Path_MSG.h>
#include <custom_messages/DJI_Bridge_Travel_Speed_MSG.h>
#include <custom_messages/Costmap_Bridge_Travel_Path_MSG.h>

// talk to the planner
#include <custom_messages/Costmap_Bridge_Status_MSG.h>
#include <custom_messages/Costmap_Bridge_Team_Map_Update_MSG.h>

// services I offer
#include "custom_messages/Get_A_Star_Path.h"
#include "custom_messages/Get_Kinematic_A_Star_Path.h"

// normal c++ libs
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>
#include <math.h>

// my other file
#include "Costmap_Utils.h"


using namespace std;
using namespace cv;

class Costmap{
public:

	Costmap(ros::NodeHandle nh, const int &test_environment_number, const int &agent_index, const int &jetson, const int &param_seed);
	~Costmap();
	// services
	ros::ServiceServer a_star_path_server, kinematic_path_server;
	bool a_star_path_server_callback(custom_messages::Get_A_Star_Path::Request &req, custom_messages::Get_A_Star_Path::Response &resp);	
	bool kinematic_path_server_callback(custom_messages::Get_Kinematic_A_Star_Path::Request &req, custom_messages::Get_Kinematic_A_Star_Path::Response &resp);
	
	// subscribe to zed costmap
	ros::Subscriber costmap_subscriber;
	void costmap_callback(const nav_msgs::OccupancyGrid& cost_in );
	// proivde the rest of the team with my observations
	ros::Publisher costmap_update_publisher;
	void publish_map_updates(const std::vector<cv::Point> &u_pts, const std::vector<int> &u_types);
	// take in other people's updates and include them
	ros::Subscriber costmap_update_subscriber;
	void costmap_update_callback( const custom_messages::Costmap_Bridge_Team_Map_Update_MSG& updates );
	// subscribe to DJI_bridge
	ros::Subscriber quad_status_subscriber;
	void DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in);
	// subscribe to the planner
	ros::Subscriber dist_planner_goal_subscriber;
	void dist_planner_goal_callback( const custom_messages::Costmap_Bridge_Travel_Path_MSG& status_in);
	// publish to dji_bridge
	ros::Publisher path_publisher;
	void publish_travel_path(const std::vector<Point2d> &path);
	void find_path_and_publish();
	bool find_path(std::vector<Point> &cells_path);
	// publish to the planner and dji_bridge
	ros::Publisher status_publisher;
	void publish_Costmap_Bridge_Status();
	// publish stuff to RVIZ
	ros::Publisher marker_publisher;
	void publish_rviz_path(const std::vector<Point2d> &path);
	visualization_msgs::Marker makeRvizMarker(const Point2d &loc, const double &radius, const int &color, const int &id);

	bool travelling, waiting, emergency_stopped;

	int agent_index;

	// map metadata
	double meters_per_cell;
	Point map_offset;
	Point2d map_offset_meters;
	Point2d map_local_origin;

	// costmap planning stuff
	Point cell_loc, cell_goal;
	Point2d local_loc, local_goal, published_local_goal;
	bool locationInitialized, costmapInitialized;

	double travelSpeed; // my travelling speed
	double set_alt;
	
	ros::Time act_time, plot_time, status_time;
	ros::Duration act_interval, plot_interval, status_interval;
	
	// working
	ros::Time actTimer;

	std::vector<cv::Point2d> wp_path;
	vector<cv::Point> cells_path;

	// costmap class stuff
	Costmap_Utils* utils;
};

#endif /* SRC_Costmap_H_ */
