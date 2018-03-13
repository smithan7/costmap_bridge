/*
 * Costmap.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef COSTMAP_UTILS_H_
#define COSTMAP_UTILS_H_

// ros stuff
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

// c++ stuff
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

//using namespace std;
//using namespace cv;

#include "State.h"

class Costmap;

class Costmap_Utils {

public:
	Costmap_Utils(Costmap* costmap);
	virtual ~Costmap_Utils();


	bool a_star_path(const cv::Point &sLoc, const cv::Point &gLoc, std::vector<cv::Point> &path, double &length);
	void cells_to_local(const cv::Point &cell, cv::Point2d &loc);
	void display_costmap(); // show nice display plot and number it
	void build_display_plot(); // build nice display plot
	void add_agent_to_costmap_plot(const cv::Scalar &color, const std::vector<cv::Point> &myPath, const cv::Point &cLoc);

	void global_to_local(const double &lat_in, const double &lon_in, double &x_out, double &y_out); // gps to local x/y from origin which is the center of the map
	void update_cells(const nav_msgs::OccupancyGrid& cost_in); // update cells with observation
	void team_map_update( const std::vector<int> &xs, const std::vector<int> &ys, const std::vector<int> &ts); // used to share updates with team
	void local_to_cells(const cv::Point2d &loc, cv::Point &cell);
	bool point_in_cells(const cv::Point &p);

	// true util functions
	double get_global_distance(const double &lata, const double &lona, const double &latb, const double &lonb);
	double get_global_heading(const double &lata, const double &lona, const double &latb, const double &lonb);
	double to_radians(const double &deg);
	double to_degrees(const double &rad);

	// for putting the path together
	double get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2);
	double get_local_euclidian_distance(const cv::Point2d &l1, const cv::Point2d &l2);
	void cells_to_local_path(const std::vector<cv::Point> &cells_path, std::vector<cv::Point2d> &local_path);

	cv::Mat cells; // holds occupancy
	cv::Mat displayPlot; // for plotting cells
	int map_width_cells, map_height_cells;
	double map_width_meters, map_height_meters;
	double cells_per_meter, meters_per_cell;
	cv::Point offset_cells;

private:
	// useful stuff
	Costmap* costmap;
	bool pay_obstacle_costs, use_gazebo; // am I affected by obstacles?
	std::string test_environment_img, test_obstacle_img;

	double north_lat, south_lat, east_lon, west_lon, origin_lat, origin_lon;
	
	// values 
	int obsFree, infFree, obsOccupied, infOccupied;
	int ros_occupied, ros_free, ros_unknown;
	cv::Vec3b cObsFree, cInfFree, cObsOccupied, cInfOccupied;
	double obsFree_cost, infFree_cost, obsOcc_cost, infOcc_cost;
	bool need_initialization;
	std::vector<cv::Point2d> starting_locs;
	int n_obstacles;

	// 1 = free space // 2 = inferred free space // 3 = domFree
	// 101 = unknown
	// 201 = wall // 202 = inferred wall // 203 = inflated wall

	// functions
    // use satelite info to seed the exploration
    cv::Mat Obs_Mat, Env_Mat;
	std::vector< std::vector<double> > obstacles;
	int rand_seed;
	void create_obs_mat();
	void get_obs_mat();
	void build_cells_mat();

	// used to get from occ_grid array to cells
	cv::Point get_cell_index(const int &l);

	void cells_path_to_local_path(const std::vector<cv::Point> &cp, std::vector<cv::Point2d> &lp);
	void local_to_global(const cv::Point2d &local, cv::Point2d &global);

	// distances and planning
	double get_cells_euclidian_distance(const cv::Point &a, const cv::Point &b);
	double rand_double_in_range(const double &min, const double &max);

	void test_a_star_planner(const cv::Point &s, const cv::Point &g);
	double get_occ_penalty(const cv::Point &p); // get the occupancy at p
	double a_star_heuristic_weight, euclid_threshold;

	int inflation_iters;

	double get_occ_penalty(State &state);
	bool state_to_bin( State &s, cv::Point &b);
};

#endif /* COSTMAP_UTILS_H_ */
