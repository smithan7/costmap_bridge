/*
 * Costmap.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Costmap.h"
#include "State.h"


using namespace std;
using namespace cv;

Costmap::Costmap(ros::NodeHandle nHandle){

    std::vector<bool> team_pay_obstacle_costs;
	ROS_INFO("Costmap Bridge::Costmap::Costmap: loading rosparams");
    std::string pkg_directory;
    double inflation_box_size_meters;
	ros::param::get("/package_directory", pkg_directory);
	ros::param::get("/test_environment_img", this->test_environment_img);
	ros::param::get("/test_obstacle_img", this->test_obstacle_img);
	ros::param::get("/agent_index",this->agent_index);
	ros::param::get("/param_number", this->param_seed);
	ros::param::get("/pay_obs_costs", team_pay_obstacle_costs);
	ros::param::get("/north_lat", this->north_lat);
	ros::param::get("/south_lat", this->south_lat);
	ros::param::get("/east_lon", this->east_lon);
	ros::param::get("/west_lon", this->west_lon);
	ros::param::get("/meters_per_cell", this->meters_per_cell);
	ros::param::get("/display_costmap_path", this->display_costmap);
	ros::param::get("/inflation_iters", this->inflation_iters);
	ros::param::get("/inflation_box_size_meters", inflation_box_size_meters);
	ros::param::get("/inflation_sigma", this->inflation_sigma);
	ros::param::get("/use_gazebo", this->use_gazebo);
	ros::param::get("/starting_xs", this->starting_xs);
	ros::param::get("/starting_ys", this->starting_ys);

    this->pay_obstacle_costs = team_pay_obstacle_costs[this->agent_index];
    this->test_obstacle_img = pkg_directory + this->test_obstacle_img;
    this->test_environment_img = pkg_directory + this->test_environment_img;

	this->origin_lat = (this->north_lat + this->south_lat)/2.0;
	this->origin_lon = (this->east_lon + this->west_lon)/2.0;

	this->cells_per_meter = 1.0 / this->meters_per_cell;
    this->inflation_box_size = inflation_box_size_meters * this->cells_per_meter;

	ROS_INFO("Costmap Bridge::Costmap::Costmap: got rosparams");
	ROS_INFO("   package directory %s", pkg_directory.c_str());
	ROS_INFO("   test_environment_img %s", this->test_environment_img.c_str());
	ROS_INFO("   test_obstacle_img %s", this->test_obstacle_img.c_str());
	ROS_INFO("   agent_index %i", this->agent_index);
	ROS_INFO("   parameter_seed %i", this->param_seed);
	ROS_INFO("   pay_obstacle_costs %i", this->pay_obstacle_costs);
    ROS_INFO("   north_lat %0.6f", this->north_lat);
	ROS_INFO("   south_lat %0.6f", this->south_lat);
	ROS_INFO("   west_lon %0.6f", this->west_lon);
	ROS_INFO("   east_lon %0.6f", this->east_lon);
	ROS_INFO("   origin_lat %0.6f", this->origin_lat);
	ROS_INFO("   origin_lon %0.6f", this->origin_lon);
	ROS_INFO("   meters_per_cell %0.2f", this->meters_per_cell);
	ROS_INFO("   display_costmap %i", this->display_costmap);
	ROS_INFO("   inflation_iters %i", this->inflation_iters);
	ROS_INFO("   inflation_box_size_meters %0.2f", inflation_box_size_meters);
	ROS_INFO("   inflation_box_size %0.2f", this->inflation_box_size);
	ROS_INFO("   inflation_sigma %0.2f", this->inflation_sigma);
	ROS_INFO("   use_gazebo %i", this->use_gazebo);
	ROS_INFO("   starting_xs.size(): %i", int(this->starting_xs.size()));
	ROS_INFO("   starting_ys.size(): %i", int(this->starting_ys.size()));


	// initialize cell locs
	this->cell_loc= Point(-1,-1);
	this->cell_goal = Point(50,50);
	// initialize local locs
	this->local_loc = Point2d(-1.0, -1.0);
	this->local_goal = Point2d(50.0, 50.0);
	this->published_local_goal = Point2d(-1, -1);
	// set initial flags false
	this->locationInitialized = false;
	this->costmapInitialized = false;

	this->travelling = false;
	this->waiting = false;
	this->emergency_stopped = false;

	this->status_time = ros::Time::now(); // when did I last publish a status report
	this->status_interval = ros::Duration(1.0);
	this->act_time = ros::Time::now();
	this->act_interval = ros::Duration(1.0); // how often should I replan if I don't get an update or request
	this->plot_time = ros::Time::now(); // when did I last display the plot
	this->plot_interval = ros::Duration(1.0); // plot at 1 Hz
	
	this->set_alt = 0.0;
	this->travelSpeed = 0.0;
	this->cells_per_meter = 1.0/ meters_per_cell;
	this->n_obstacles = 10;

	/////////////////////// Subscribers /////////////////////////////
	// update the costmap
	//this->costmap_subscriber = nHandle.subscribe("/rtabmap/grid_map", 1, &Costmap::costmap_callback, this);
	// get team member map updates
	//this->costmap_update_subscriber = nHandle.subscribe("/team_map_update", 1, &Costmap::costmap_update_callback, this);
	// quad status callback
	//this->quad_status_subscriber = nHandle.subscribe("/dji_bridge_status", 1, &Costmap::DJI_Bridge_status_callback, this);
	// get goal from Dist MCTS Goal callback
	//this->dist_planner_goal_subscriber = nHandle.subscribe("/wp_travel_path", 1, &Costmap::dist_planner_goal_callback, this);

	/////////////////////// Publishers //////////////////////////////
	// provide team with my observations
	//this->costmap_update_publisher = nHandle.advertise<custom_messages::Costmap_Bridge_Team_Map_Update_MSG>("/team_map_update", 1);
	// tell the DJI Bridge where I am going
	//this->path_publisher =  nHandle.advertise<custom_messages::DJI_Bridge_Travel_Path_MSG>("/travel_path", 1);
	// tell everyone my status
	//this->status_publisher = nHandle.advertise<custom_messages::Costmap_Bridge_Status_MSG>("/costmap_bridge_status", 1);
	// publish marker to RVIZ
	this->marker_publisher = nHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

	// initialize world services
	char temp[200];
	int n = sprintf(temp, "/dmcts_%i/costmap_bridge/a_star_path", this->agent_index);
	this->a_star_path_server = nHandle.advertiseService(temp, &Costmap::a_star_path_server_callback, this);
	
	this->a_star_path_request_subscriber = nHandle.subscribe("/costmap/a_star_path_request", 1, &Costmap::a_star_path_sub_callback, this);
	this->a_star_path_request_publisher = nHandle.advertise<nav_msgs::Path>("/costmap/a_star_path_response", 1);

	// really initialize costmap
	this->costmapInitialized = false;
	this->utils = new Costmap_Utils(this);
}

Costmap::~Costmap(){
	delete this->utils;
}


void Costmap::a_star_path_sub_callback( const nav_msgs::Path &req_in){
    cv::Point2d ls(double(req_in.poses[0].pose.position.x), double(req_in.poses[0].pose.position.y));
	cv::Point s;
	this->utils->local_to_cells(ls, s);
	cv::Point2d lg(double(req_in.poses[1].pose.position.x), double(req_in.poses[1].pose.position.y));
	cv::Point g;
	//ROS_INFO("Costmap::a_star_path_server_callback: s: %i, %i", s.x, s.y);
	this->utils->local_to_cells(lg, g);
	//ROS_INFO("Costmap::a_star_path_server_callback: g: %i, %i", g.x, g.y);
	std::vector<cv::Point> path;
	double length = 0.0;
	
	if(this->display_costmap){
	    cv::Mat tst = this->utils->displayPlot.clone();
	    cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
	    cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

	    cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
	    cv::imshow("a_star_path", tst);
	    cv::waitKey(100);
	}
	
	if(this->utils->a_star_path(s,g,path,length)){
		nav_msgs::Path path_out;
		
		for(size_t i=0; i<path.size(); i++){
			cv::Point2d l;
			cv::Point ll(path[i].x, path[i].y);
			this->utils->cells_to_local(ll, l);
			//resp.xs.push_back(l.x);
			//resp.ys.push_back(l.y);
			
			geometry_msgs::PoseStamped pp;
			pp.pose.position.x = l.x;
			pp.pose.position.y = l.y;
			
			path_out.poses.push_back(pp);
			
		}
		//resp.path_length = length;
		//resp.success = true;
		
		if(this->display_costmap){
		    cv::Mat tst = this->utils->displayPlot.clone();
		    cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
		    cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

		    for(size_t i=0; i<path.size(); i++){
			    cv::circle(tst, path[i], 1, cv::Scalar(255,0,0), -1);
		    }

		    cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
		    cv::imshow("a_star_path", tst);
		    cv::waitKey(100);
		}
		
		this->a_star_path_request_publisher.publish(path_out);
		
	}
	else{
		ROS_WARN("Costmap::a_star_path_server_callback: failed to find path");
	}
}

bool Costmap::a_star_path_server_callback(custom_messages::Get_A_Star_Path::Request &req, custom_messages::Get_A_Star_Path::Response &resp){
	/*
	int32 start_x
	int32 start_y
	int32 goal_x
	int32 goal_y
	int32 map_num
	---
	bool success
	float64[] xs
	float64[] ys
	float64 path_length
	*/
	
	//ROS_INFO("Costmap::a_star_path_server_callback: req.start: %.2f, %.2f", req.start_x, req.start_y);
	//ROS_INFO("Costmap::a_star_path_server_callback: req.goal: %.2f, %.2f", req.goal_x, req.goal_y);
	cv::Point2d ls(double(req.start_x), double(req.start_y));
	cv::Point s;
	this->utils->local_to_cells(ls, s);
	cv::Point2d lg(double(req.goal_x), double(req.goal_y));
	cv::Point g;
	//ROS_INFO("Costmap::a_star_path_server_callback: s: %i, %i", s.x, s.y);
	this->utils->local_to_cells(lg, g);
	//ROS_INFO("Costmap::a_star_path_server_callback: g: %i, %i", g.x, g.y);
	std::vector<cv::Point> path;
	double length = 0.0;
	
	if(this->display_costmap){
	    cv::Mat tst = this->utils->displayPlot.clone();
	    cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
	    cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

	    cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
	    cv::imshow("a_star_path", tst);
	    cv::waitKey(100);
	}
	
	if(this->utils->a_star_path(s,g,path,length)){
		for(size_t i=0; i<path.size(); i++){
			cv::Point2d l;
			cv::Point ll(path[i].x, path[i].y);
			this->utils->cells_to_local(ll, l);
			resp.xs.push_back(l.x);
			resp.ys.push_back(l.y);
		}
		resp.path_length = length;
		resp.success = true;
		
		if(this->display_costmap){
		    cv::Mat tst = this->utils->displayPlot.clone();
		    cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
		    cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

		    for(size_t i=0; i<path.size(); i++){
			    cv::circle(tst, path[i], 1, cv::Scalar(255,0,0), -1);
		    }

		    cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
		    cv::imshow("a_star_path", tst);
		    cv::waitKey(100);
		}
		return true;
		
	}
	else{
		resp.path_length = INFINITY;
		resp.success = false;
		ROS_WARN("Costmap::a_star_path_server_callback: failed to find path");
		return true;
	}
}


void Costmap::costmap_callback(const nav_msgs::OccupancyGrid& cost_in ){
	// update my costmap
	//ROS_INFO("updating costmap");
	this->utils->update_cells( cost_in );
	this->costmapInitialized = true;
	// plan path on costmap and publish it to the quad
	//ROS_INFO("planning path");
	//this->find_path_and_publish();

}

void Costmap::publish_map_updates(const std::vector<cv::Point> &u_pts, const std::vector<int> &u_types){
	// convert to message format
	custom_messages::Costmap_Bridge_Team_Map_Update_MSG tmu;
	for(size_t i=0; i<u_pts.size(); i++){
		tmu.xs.push_back(u_pts[i].x);
		tmu.ys.push_back(u_pts[i].y);
		tmu.tps.push_back(u_types[i]);	
	}
	// publish
	costmap_update_publisher.publish(tmu);
}

void Costmap::costmap_update_callback( const custom_messages::Costmap_Bridge_Team_Map_Update_MSG &update){
	if( !this->costmapInitialized ){ // have i initialized the costmap?
		ROS_ERROR("Costmap_Bridge::Costmap::costmap_callback::costmap not initialized");
		return;
	}


	std::vector<int> xs = update.xs;
	std::vector<int> ys = update.ys;
	std::vector<int> tps = update.tps;
	
	this->utils->team_map_update(xs, ys, tps);
}

void Costmap::DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in){	
	
	double x,y;
	this->utils->global_to_local(status_in.latitude, status_in.longitude, x,y);
	cv::Point2d l(x,y);
	cv:;Point c;
	this->utils->local_to_cells(l, c);
	//ROS_WARN("g: %0.6f, %0.6f", g.x, g.y);
	//ROS_WARN("l: %0.2f, %0.2f", l.x, l.y);

	if(this->utils->point_in_cells(c)){
		this->cell_loc = c;
		this->local_loc = l;
		this->locationInitialized = true;	
	}
	else{
		this->locationInitialized = false;
		ROS_ERROR("Costmap_Bridge::Costmap::DJI_Bridge_Status::off map");
		ROS_ERROR("     cells.size( %i, %i ) and point (%i, %i)", this->utils->cells.rows, this->utils->cells.cols, c.x, c.y);		
		return;	
	}

	if(ros::Time::now() - this->plot_time > this->plot_interval){
		this->plot_time = ros::Time::now();

		this->utils->build_display_plot();
		Scalar blue = Scalar(255,0,0);
		this->utils->add_agent_to_costmap_plot( blue, this->cells_path, this->cell_loc);
		Scalar orange = Scalar(0,165,255);
		this->utils->add_agent_to_costmap_plot( orange, this->cells_path, this->cell_goal);
		this->utils->display_costmap();
	}

	if(ros::Time::now() - this->status_time > this->status_interval){
		this->status_time = ros::Time::now();
		publish_Costmap_Bridge_Status();
	}
	this->find_path_and_publish();
}

void Costmap::dist_planner_goal_callback( const custom_messages::Costmap_Bridge_Travel_Path_MSG& path_in){
	ROS_INFO("Costmap_Bridge::Dist_Planner_Callback:: %i Wps recieved", int(path_in.local_xs.size() ) );		
	this->wp_path.clear();
	this->cells_path.clear();
	this->set_alt = std::max(10.0, path_in.altitude);
	for(size_t i=0; i<path_in.local_xs.size(); i++){
		Point2d l_wp(path_in.local_xs[i], path_in.local_ys[i]);
		Point c_wp;
		this->utils->local_to_cells(l_wp, c_wp);

		if(!this->utils->point_in_cells(c_wp)){
			ROS_ERROR("Costmap_Bridge::Dist_Planner_Callback::Wp off map");
			return;
		}

		this->wp_path.push_back(l_wp);
		this->cells_path.push_back(c_wp);
	}

	this->local_goal = this->wp_path.back();
	this->cell_goal = this->cells_path.back();
	ROS_INFO("Costmap_Bridge::Dist_Planner_Callback:: %i Wps recieved", int(this->wp_path.size() ) );
	this->find_path_and_publish();
}

void Costmap::publish_travel_path(const std::vector<Point2d> &path){
	// initialize msg
	custom_messages::DJI_Bridge_Travel_Path_MSG path_msg;
	cv::Point2d local, next_local;
	// for length of path
	for(size_t i=0; i<path.size(); i++){
		// go from cells to loc_x, loc_y
		// set path
		path_msg.local_xs.push_back(double(local.x));
		path_msg.local_ys.push_back(double(local.y));
		path_msg.altitudes.push_back(this->set_alt);
		// handle heading
		if(i < path.size() - 1){
			// if not last point, heading is from current to next
			next_local = path[i+1];
			double heading = this->utils->get_local_heading(next_local, local);
			path_msg.headings.push_back(heading);
		}
		else{
			// last point, keep current heading
			path_msg.headings.push_back(path_msg.headings.back());
		}
		// set local as next_local and move to next pt
		local = next_local;
	}
	// publish path
	this->path_publisher.publish(path_msg);
}

void Costmap::find_path_and_publish(){
	if( ros::Time::now() - this->actTimer > this->act_interval || this->published_local_goal != this->local_goal ){
		this->published_local_goal = this->local_goal;
		this->actTimer = ros::Time::now();

		bool flag = false;
		if( this->locationInitialized ){
			if( this->costmapInitialized || true){
				ROS_WARN("Costmap::act::costmap check removed");
				flag = true;
			}
			else{
				ROS_WARN("Costmap::act::waiting on costmap callback");
			}
		}
		else{
			ROS_WARN("Costmap::act::waiting on location callback");
		}

        if( flag ){
        	ROS_INFO("Costmap::act::publishing path to quad");

					
			//this->wp_path.clear();
			//ROS_WARN("Costmap_Bridge::Costmap::Fake Goal");
			//this->wp_path.push_back(this->local_goal); // this nneds to be ERASED for trials
			//this->wp_path.push_back(cv::Point(125,70));
			//this->utils->local_to_cells(this->local_goal, this->cell_goal);
			//ROS_INFO("Costmap_Bridge::Costmap::local_goal: %.2f, %.2f", this->local_goal.x, this->local_goal.y);
			//ROS_INFO("Costmap_Bridge::Costmap::local_loc: %.2f, %.2f", this->local_loc.x, this->local_loc.y);
						
			//ROS_INFO("Costmap_Bridge::Costmap::cell_goal: %i, %i", int(this->cell_goal.x), int(this->cell_goal.y));
			//ROS_INFO("Costmap_Bridge::Costmap::cell_loc: %i, %i", int(this->cell_loc.x), int(this->cell_loc.y));
			
			if(this->find_path(this->cells_path)){
				ROS_INFO("Costmap_Bridge::Costmap::published path length: %i", int(this->cells_path.size()));
        		std::vector<Point2d> local_path;
        		this->utils->cells_to_local_path(this->cells_path, local_path);

				// assemble plot and display it
				this->utils->build_display_plot();
				Scalar blue = Scalar(255,0,0);
				this->utils->add_agent_to_costmap_plot( blue, this->cells_path, this->cell_loc);
				Scalar orange = Scalar(0,165,255);
				this->utils->add_agent_to_costmap_plot( orange, this->cells_path, this->cell_goal);
				this->utils->display_costmap();

        		this->publish_travel_path(local_path);
        		this->publish_rviz_path(local_path);	
        	}
			else{
				ROS_WARN("Costmap Bridge::Costmap::find path and publish: could not find path");
			}
		}
	}
}

bool Costmap::find_path( std::vector<cv::Point> &cells_path ){

	// ensure starting fresh
	cells_path.clear();
	// convert wp_path to cells wp
	Point s_wp_cells = this->cell_loc;
	for(size_t i=0; i<this->wp_path.size(); i++){
		// get end point in cells space
		Point e_wp_cells;
		this->utils->local_to_cells(this->wp_path[i], e_wp_cells);
		std::vector<Point> cp;
		double length=0.0;
		if(this->utils->a_star_path(s_wp_cells, e_wp_cells, cp, length)){
			// a* found a path 
			if(cp.size() > 1){
				cells_path.insert(cells_path.end(), cp.begin()+1, cp.end());
			}
			if(cells_path.size() > 100){
				cells_path.erase(cells_path.begin()+100, cells_path.end());
				return true;
			}
		}
		else{
			// a* could not find a path, will also return path to current point
			return false;
		}
		// seed in next starting point as current point
		s_wp_cells = e_wp_cells;
	}
	return true;
}

void Costmap::publish_Costmap_Bridge_Status(){
	custom_messages::Costmap_Bridge_Status_MSG msg;
	msg.longitude = this->local_loc.x;
	msg.latitude = this->local_loc.y;
	msg.altitude = this->set_alt;
	msg.heading = 0.0;
	msg.goal_longitude = this->local_goal.x;
	msg.goal_latitude = this->local_goal.y;

	msg.travelling = this->travelling;
	msg.emergency_stopped = this->emergency_stopped;
	msg.waiting = this->waiting;
	this->status_publisher.publish(msg);
}

void Costmap::publish_rviz_path(const std::vector<Point2d> &path){
	visualization_msgs::MarkerArray marker_array;

	int marker_cntr = 0;

	for(size_t i=0; i<path.size(); i++){
		visualization_msgs::Marker marker = makeRvizMarker(path[i], 2.0, 1, marker_cntr);
		marker_array.markers.push_back(marker);		
		marker_cntr++;
	}
	visualization_msgs::Marker marker = makeRvizMarker(this->local_loc, 5.0, 2, marker_cntr);
	marker_array.markers.push_back(marker);
	marker_cntr++;

	for(size_t i=0; i<this->wp_path.size(); i++){
		visualization_msgs::Marker marker = makeRvizMarker(this->wp_path[i], 5.0, 0, marker_cntr);
		marker_array.markers.push_back(marker);
		marker_cntr++;
	}
	
	this->marker_publisher.publish(marker_array);
}

visualization_msgs::Marker Costmap::makeRvizMarker(const Point2d &loc, const double &radius, const int &color, const int &id){
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = loc.x;
    marker.pose.position.y = loc.y;
    marker.pose.position.z = 1.5;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    if(color == 0){
    	marker.color.r = 1.0f;
    }
    else if(color == 1){
    	marker.color.g = 1.0f;
    }
    else if(color == 2){
    	marker.color.b = 1.0f;
    }

    marker.lifetime = ros::Duration(1);
	return marker;    
}
