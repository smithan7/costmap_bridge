/*
 * Costmap.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */




#include "Costmap_Utils.h"
#include "Costmap.h"

bool pointCompare(const cv::Point &a, const cv::Point &b);
bool pointOnMat(const cv::Point &a, const cv::Mat &b);
std::vector<cv::Point> get_image_points_at_intensity(const cv::Mat &image, const int &intensity);
double lin_interp(const double &p_min, const double &p_max, const double &p);


Costmap_Utils::Costmap_Utils(Costmap* costmap){

	this->euclid_threshold = 1.0;

	// I need to be initialized
	this->need_initialization = true;
	this->costmap = costmap;
	this->rand_seed = costmap->param_seed;
	this->test_environment_img = costmap->test_environment_img;
	this->test_obstacle_img = costmap->test_obstacle_img;
	this->n_obstacles = costmap->n_obstacles;
	this->use_gazebo = costmap->use_gazebo;
	this->starting_xs = costmap->starting_xs;
	this->starting_ys = costmap->starting_ys;

	// set heuristic for A*
	this->a_star_heuristic_weight = 1.0;//2.75; // 1->inf get greedier
	this->inflation_iters = costmap->inflation_iters;
	this->inflation_box_size = costmap->inflation_box_size;
	this->inflation_sigma = costmap->inflation_sigma;

	// ros's occupancy grid values
	this->ros_unknown = -1;
	this->ros_occupied = 100;
	this->ros_free = 0;

	// values I use to track everything
	this->obsFree = 1;
	this->infFree = 2;
	this->obsOccupied = 201;
	this->infOccupied = 202;

	// travel costs
	this->obsOcc_cost = 200.0;
	this->infOcc_cost = 100.0;
	this->obsFree_cost = 0.0;
	this->infFree_cost = 0.5;

	// do I pay obstacles
	this->pay_obstacle_costs = costmap->pay_obstacle_costs;

	// annoying but it works to seed plot colors
	cv::Vec3b a(255,255,255);
	this->cObsFree = a;
	a = cv::Vec3b(200,200,200);
	this->cInfFree = a;
	a = cv::Vec3b(0,0,0);
	this->cObsOccupied = a;
	a = cv::Vec3b(50,50,50);
	this->cInfOccupied = a;

	this->north_lat = costmap->north_lat;
	this->south_lat = costmap->south_lat;
	this->west_lon = costmap->west_lon;
	this->east_lon = costmap->east_lon;
	this->origin_lat = costmap->origin_lat;
	this->origin_lon = costmap->origin_lon;

   	if(this->test_obstacle_img.empty()){
    	this->map_width_meters = 100.0;
	    this->map_height_meters = 100.0;
	}
	else{
	this->map_width_meters = this->get_global_distance(this->north_lat, this->west_lon, this->north_lat, this->east_lon);
	    this->map_height_meters = this->get_global_distance(this->north_lat, this->west_lon, this->south_lat, this->west_lon);
	}
	
	if(this->use_gazebo){
		if(this->map_width_meters > this->map_height_meters){
			this->map_height_meters = 90.0 * this->map_height_meters / this->map_width_meters;
			this->map_width_meters = 90.0;
		}
		else{
			this->map_width_meters = 90.0 * this->map_width_meters / this->map_height_meters;
			this->map_height_meters = 90.0;
		}
	}
	
	ROS_INFO("    map size: %0.2f, %0.2f (meters)", this->map_width_meters, this->map_height_meters);

	// set cells per meter
	this->meters_per_cell = costmap->meters_per_cell;
	ROS_INFO("    meters per cell: %0.2f", this->meters_per_cell);
	this->cells_per_meter = 1.0 / this->meters_per_cell;

	ROS_INFO("    cells per meter: %0.2f", this->cells_per_meter);
	this->map_width_cells = ceil(this->map_width_meters * this->cells_per_meter);
	this->map_height_cells = ceil(this->map_height_meters * this->cells_per_meter);

	// initialize cells
	cv::Mat b = cv::Mat::ones( this->map_height_cells, this->map_width_cells, CV_16S)*this->infFree;
	this->cells = b.clone();
	ROS_INFO("    map size: %i, %i (cells)", this->cells.cols, this->cells.rows);
	this->offset_cells.x = this->cells.cols / 2.0;
	this->offset_cells.y = this->cells.rows / 2.0;
	
	// reset randomization
	srand(this->rand_seed);
	this->get_obs_mat(); // create random / or load obstacles
	ROS_INFO("DMCTS_world_node::   Word::World(): mat size: %i, %i (cells)", this->Obs_Mat.cols, this->Obs_Mat.rows);
	
	this->build_cells_mat();
	this->build_display_plot();
	//this->display_costmap();// show nice display plot and number it

	// announce I am initialized!
	this->need_initialization = false;
	ROS_INFO("Costmap_Utils::initialize_costmap::complete");

	//Test A Star Stuff
	//cv::Point s(50,50);
	//cv::Point g(500,50);
	//this->test_a_star_planner(s,g);

	// x,y,speed,theta
	//State s_state(5.0,5.0,0.0,1.57);
	//State g_state(65.0,95.0,0.0,0.0);
	//this->test_kinematic_a_star_planner(s_state, g_state);
	//this->test_vxvy_a_star_planner(s_state, g_state);
}

double Costmap_Utils::rand_double_in_range(const double &min, const double &max) {
	// get random double between min and max
	return (max - min) * double(rand()) / double(RAND_MAX) + min;
}

void Costmap_Utils::build_cells_mat(){
	cv::resize(this->Obs_Mat, this->Obs_Mat, this->cells.size());
	// go through every pixel of the image and occupancy map
	for(int i=0; i<this->Obs_Mat.cols; i++){
		for(int j=0; j<this->Obs_Mat.rows; j++){
			cv::Point p(i,j);
			if(this->Obs_Mat.at<uchar>(p) == 255){
				this->cells.at<short>(p) = 255;
			}
			else{
				this->cells.at<short>(p) = int(100.0 * (double(this->Obs_Mat.at<uchar>(p))/255.0));
			}
		}
	}
}

void Costmap_Utils::get_obs_mat(){

	this->Obs_Mat = cv::Mat::zeros(cv::Size(int(this->map_width_meters*this->cells_per_meter), int(this->map_height_meters*this->cells_per_meter)), CV_8UC1);
	this->Env_Mat = cv::Mat::zeros(cv::Size(int(this->map_width_meters*this->cells_per_meter), int(this->map_height_meters*this->cells_per_meter)), CV_8UC3);

	cv::Mat temp_obs = cv::imread(this->test_obstacle_img, CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat temp_env = cv::imread(this->test_environment_img, CV_LOAD_IMAGE_COLOR);

	//cv::namedWindow("DMCTS_World::World::seed_obs_mat:Obstacles", cv::WINDOW_NORMAL);
	//cv::imshow("DMCTS_World::World::seed_obs_mat:Obstacles", temp_env);
	//cv::waitKey(0);

	if(!temp_obs.data || !temp_env.data){
		this->create_obs_mat();
		ROS_WARN("World::seed_img::Could NOT load img, creating img");
		return;
	}
	else{
	    cv::Mat temp;               // dst must be a different Mat
	    //cv::flip(temp_obs, temp, 1); 
		cv::resize(temp_obs, this->Obs_Mat, this->Obs_Mat.size());
		//cv::flip(temp_env, temp, 1);
		cv::resize(temp_env, this->Env_Mat, this->Env_Mat.size());
	}

	cv::Mat s = cv::Mat::zeros(this->Obs_Mat.size(), CV_8UC1);
	for(int i=0; i<this->inflation_iters; i++){
		cv::GaussianBlur(this->Obs_Mat,s,cv::Size(this->inflation_box_size, this->inflation_box_size), this->inflation_sigma, this->inflation_sigma);
		cv::max(this->Obs_Mat,s,this->Obs_Mat);
	}

	//cv::namedWindow("Costmap_Utils::get_obs_mat:Obstacles", cv::WINDOW_NORMAL);
	//cv::imshow("Costmap_Utils::get_obs_mat:Obstacles", this->Obs_Mat);
	//cv::waitKey(10);

	//cv::namedWindow("DMCTS_World::World::seed_obs_mat:Env_Mat", cv::WINDOW_NORMAL);
	//cv::imshow("DMCTS_World::World::seed_obs_mat:Env_Mat", this->Env_Mat);
	//cv::waitKey(0);
}

void Costmap_Utils::create_obs_mat(){

	this->map_width_meters = 100.0;
	this->map_height_meters = 100.0;
	this->Obs_Mat = cv::Mat::zeros(cv::Size(int(this->map_width_meters*this->cells_per_meter), int(this->map_height_meters*this->cells_per_meter)), CV_8UC1);
	this->Env_Mat = cv::Mat::zeros(cv::Size(int(this->map_width_meters*this->cells_per_meter), int(this->map_height_meters*this->cells_per_meter)), CV_8UC3);
	
	this->obstacles.clear();
	//ROS_INFO("DMCTS_World::World::make_obs_mat: making obstacles");
	while(this->obstacles.size() < this->n_obstacles){
		//ROS_INFO("making obstacle");
		// create a potnetial obstacle
		double rr = rand_double_in_range(1,10);
		double xx = rand_double_in_range(-this->map_width_meters/2.1,this->map_width_meters/2.1);
		double yy = rand_double_in_range(-this->map_height_meters/2.1,this->map_height_meters/2.1);
		//ROS_INFO("obs: %.1f, %.1f, r =  %.1f", xx, yy, rr);
		// check if any starting locations are in an obstacle
		bool flag = true;
		for(size_t s=0; s<this->starting_xs.size(); s++){
			double d = sqrt(pow(xx-this->starting_xs[s],2) + pow(yy-this->starting_ys[s],2));
			//ROS_INFO("starting_locs: %.1f, %.1f, d = %.1f", this->starting_xs[s]+this->map_width_meters/2, this->starting_ys[s]+this->map_height_meters/2, d);
			if(rr+2 >= d ){
				// starting loc is in obstacle
				flag = false;
				break;
			}
		}

		if(flag){
			for(size_t s=0; s<this->obstacles.size(); s++){
				double d = sqrt(pow(xx-this->obstacles[s][0],2) + pow(yy-this->obstacles[s][1],2));
				if(rr + this->obstacles[s][2]+1 >= d){
					// obstacle is in obstacle so don't make
					flag = false;
					break;
				}
			}			
		}
		if(flag){
			std::vector<double> temp = {xx,yy,rr};
			this->obstacles.push_back(temp);
		}
	}

	for(size_t i=0; i<this->obstacles.size(); i++){
		cv::circle(this->Obs_Mat, cv::Point((this->obstacles[i][0]+this->map_width_meters/2), (this->obstacles[i][1]+this->map_height_meters/2)), this->obstacles[i][2], cv::Scalar(255), -1);
		cv::circle(this->Env_Mat, cv::Point((this->obstacles[i][0]+this->map_width_meters/2), (this->obstacles[i][1]+this->map_height_meters/2)), this->obstacles[i][2], cv::Scalar(255,0,0), -1);
	}

	//cv::namedWindow("DMCTS_World::World::make_obs_mat:Obstacles", cv::WINDOW_NORMAL);
	//cv::imshow("DMCTS_World::World::make_obs_mat:Obstacles", this->Obs_Mat);
	//cv::waitKey(0);
}

void Costmap_Utils::test_a_star_planner(const cv::Point &s, const cv::Point &g){
	cv::Mat tst = this->displayPlot.clone();
	
	cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
	cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

	std::vector<cv::Point> path; 
	double length = 0.0;
	this->a_star_path(s,g, path, length);

	for(size_t i=0; i<path.size(); i++){
		cv::circle(tst, path[i], 1, cv::Scalar(255,0,0), -1);		
	}

	cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
	cv::imshow("a_star_path", tst);
	cv::waitKey(100);
	
}

Costmap_Utils::~Costmap_Utils() {}

void Costmap_Utils::update_cells( const nav_msgs::OccupancyGrid& cost_in){
	// if I haven't been initialized then don't include
	if( this->need_initialization){//  || this->cells_per_meter != res){
		return;	
	}

	int width = cost_in.info.width; // number of cells wide
	int height = cost_in.info.height; // number of cells tall
	double res = cost_in.info.resolution; // m/cell
	cv::Point2d origin; // where is the origin in meters
	origin.x = cost_in.info.origin.position.x;
	origin.y = cost_in.info.origin.position.y;

	cv::Mat t = cv::Mat::zeros(this->cells.size(), CV_8UC1);
	for(size_t i=0; i<cost_in.data.size(); i++){
		// point in the array
		cv::Point p_a( i % width, floor( i / height) );
		// point in location
		cv::Point2d p_l(double(p_a.x) * res + double(origin.x), double(p_a.y) * res + double(origin.y));
		// point in the costmap
		cv::Point p_c;
		this->local_to_cells(p_l, p_c);

		/*
		ROS_INFO("data length: %i", int(cost_in.data.size()));
		ROS_INFO("width / height: %i, / %i", width, height);
		ROS_INFO("origin: %.2f, %.2f", origin.x, origin.y);
		ROS_INFO("res (m/c): %.2f", res);
		ROS_INFO("map size (m): %.2f, %.2f", this->map_size_meters.x, this->map_size_meters.y);
		ROS_INFO("cells.size(): %i, %i", this->map_width_cells, this->map_height_cells);
		ROS_INFO("cells cols and rows: %i, %i", this->cells.cols, this->cells.rows);
		ROS_WARN("i: %i", int(i));
		ROS_WARN("p_a: %i, %i", p_a.x, p_a.y);
		ROS_WARN("p_l: %.2f, %.2f", p_l.x, p_l.y);
		ROS_WARN("p_c: %i, %i", p_c.x, p_c.y);
		cv::waitKey(0);
		*/
		if(!pointOnMat(p_c, this->cells)){
			continue;
		}

		if(cost_in.data[i] == ros_unknown){
			continue;
		}
		else if(cost_in.data[i] < ros_occupied / 4){ // ros free		 	
			//cells.at<short>(p_c) = obsFree;
		}
		else{
			//ROS_WARN("costmap add obstacles is off");
	 		//cells.at<short>(p_c) = obsOccupied;
			t.at<uchar>(p_c) = 255;
		}
	}

	// now inflate obstacles
	cv::Mat s = cv::Mat::zeros(t.size(), CV_8UC1);
	cv::blur(t,s,cv::Size(5,5));
	cv::max(t,s,t);

	//cv::namedWindow("t3", CV_WINDOW_NORMAL);
	//cv::imshow("t3", t);
	//cv::waitKey(10);

	for(int i=0; i<t.cols; i++){
		for(int j=0; j<t.rows; j++){
			cv::Point p(i,j);
			if(t.at<uchar>(p) >= 100){
				this->cells.at<short>(p) = this->obsOccupied;
			}
			else{
				this->cells.at<short>(p) = this->obsFree;
			}
		}
	}
}

void Costmap_Utils::team_map_update( const std::vector<int> &xs, const std::vector<int> &ys, const std::vector<int> &ts){

	if( this->need_initialization ){
		return;
	}

	for(size_t i=0; i<xs.size(); i++){
		cv::Point p(xs[i], ys[i]);
		if(this->point_in_cells(p)){
			if(ts[i] == this->obsFree){
			 	if( cells.at<short>(p) != obsFree){
			 		cells.at<short>(p) = obsFree;
			 	}
			}
			else if(ts[i] == this->obsOccupied){
			 	if( cells.at<short>(p) != obsOccupied){
			 		cells.at<short>(p) = obsOccupied;
			 	}
			}
		}
	}
}

 cv::Point Costmap_Utils::get_cell_index(const int &l){
 	// this is from a single vector representation of the map
	cv::Point p;
	p.x = floor( l / this->map_width_cells );
	p.y = l % this->map_height_cells;

	return p;
}

void Costmap_Utils::cells_to_local_path(const std::vector<cv::Point> &cells_path, std::vector<cv::Point2d> &local_path){
	local_path.clear();

	for(size_t i=0; i<cells_path.size(); i++){
		cv::Point2d l;
		this->cells_to_local(cells_path[i], l);
		local_path.push_back(l);
	}
}

void Costmap_Utils::cells_to_local(const cv::Point &cell, cv::Point2d &loc){
	// convert from cell to local
	loc.x = double(cell.x - this->offset_cells.x) * this->meters_per_cell;
	loc.y = double(cell.y - this->offset_cells.y) * this->meters_per_cell;
}


void Costmap_Utils::local_to_cells(const cv::Point2d &loc, cv::Point &cell){
	// move from local x/y meters to costmap cell
	cell.x = round(this->cells_per_meter * double(loc.x)) + this->offset_cells.x;
	cell.y = round(this->cells_per_meter * double(loc.y)) + this->offset_cells.y;
}

double Costmap_Utils::get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2){
	double heading = atan2(l2.x-l1.x,l2.y-l1.y);
	return heading;
}

double Costmap_Utils::get_local_euclidian_distance(const cv::Point2d &a, const cv::Point2d &b){
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double Costmap_Utils::get_cells_euclidian_distance(const cv::Point &a, const cv::Point &b){
	double dx = double(a.x - b.x);
	double dy = double(a.y - b.y);
	return sqrt(pow(dx,2) + pow(dy,2));
}





double Costmap_Utils::get_occ_penalty(State &state){
	// this finds the occupancy penalty of a continuous point by searching the discrete costmap bins, to find the correct bin and corresponding penalty
	if(!this->pay_obstacle_costs){
		return 0.0;
	}
	// get right bin
	cv::Point state_point;
	if(this->state_to_bin(state, state_point)){
		// get penalty of bin
		double occ_penalty = this->get_occ_penalty(state_point);
		// return penalty;
		return occ_penalty;
	}
	else{
		// not a valid point
		return INFINITY;
	}
}

bool Costmap_Utils::state_to_bin( State &s, cv::Point &b){
	cv::Point2d p;
	p.x = s.get_x();
	p.y = s.get_y();
	this->local_to_cells(p, b);
	return this->point_in_cells(b);
}

bool Costmap_Utils::a_star_path(const cv::Point &sLoc, const cv::Point &gLoc, std::vector<cv::Point> &path, double &length){

	if(this->need_initialization){
		return false;
	}

	// ensure that 
	path.clear();
	length = 0.0;

	if(sLoc == gLoc){
		path.clear();
		length = 0.0;
		return true;
	}

	cv::Mat cSet = cv::Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	cv::Mat oSet = cv::Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not
	cv::Mat cameFromX = cv::Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from
	cv::Mat cameFromY = cv::Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from

	cv::Mat gScore = cv::Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	cv::Mat fScore = cv::Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	std::vector<cv::Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0.0; // starting node in open set
	fScore.at<float>(sLoc) = this->a_star_heuristic_weight * this->get_cells_euclidian_distance(sLoc, gLoc);
	fScore.at<float>(gLoc) = 0.0;

	// for nbrs
	int nx[8] = {-1,-1,-1,0, 0,1,1, 1};
	int ny[8] = { 1, 0,-1,1,-1,1,0,-1};
	double neighbor_distance[8] = {1.414214, 1.0, 1.414214, 1.0, 1.0, 1.414214, 1.0, 1.414214};

	while(oVec.size() > 0){
		// find node with lowest fScore and make current
		double min = INFINITY;
		int mindex = -1;
		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}
		// I did NOT find mindex! Search over!
		if( mindex < 0){
			return false;
		}

		// I did find mindex, do maintenance
		cv::Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		// am I at the goal?
		if(pointCompare(cLoc, gLoc) ){
			// I am, construct the path 
			length = gScore.at<float>(cLoc);
			path.push_back(gLoc);
			while( cLoc.x != sLoc.x || cLoc.y != sLoc.y ){ // work backwards to start
				cv::Point tLoc(cameFromX.at<short>(cLoc), cameFromY.at<short>(cLoc));
				path.push_back(tLoc); // append path
				cLoc.x = tLoc.x;
				cLoc.y = tLoc.y;
			}
			reverse(path.begin(),path.end());
			return true;
		}

		// not at the goal, get new nbrs
		for(int ni = 0; ni<8; ni++){
			// potential nbr
			cv::Point nbr(cLoc.x + nx[ni], cLoc.y + ny[ni] );
			//ROS_WARN("nbr: %i, %i", int(nbr.x), int(nbr.y));
			// viable nbr?
			if(pointOnMat(nbr, this->cells)){
				
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}

				// calc temporary gscore, estimate of total cost
				double occ_pen = this->get_occ_penalty(nbr);
				double ngScore = gScore.at<float>(cLoc) + (1 + occ_pen) * neighbor_distance[ni]; 
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}
				cameFromX.at<short>(nbr) = cLoc.x;
				cameFromY.at<short>(nbr) = cLoc.y;

				gScore.at<float>(nbr) = ngScore;
				//ROS_WARN("ngScore: %0.3f", ngScore);

				fScore.at<float>(nbr) = gScore.at<float>(nbr) + this->a_star_heuristic_weight * this->get_cells_euclidian_distance(gLoc,nbr);

				//ROS_WARN("a* h: %0.3f", this->a_star_heuristic);
				//ROS_WARN("euclid dist: %0.3f", this->get_cells_euclidian_distance(gLoc,nbr));
				//ROS_WARN("fscore: %0.3f", gScore.at<float>(nbr) + this->a_star_heuristic * this->get_cells_euclidian_distance(gLoc,nbr));
			}
			else{
				//ROS_WARN("a star point off mat: %i, %i", int(nbr.x), int(nbr.y));
			}
		}
	}
	return false;
}

double Costmap_Utils::get_occ_penalty(const cv::Point &p){
	if(!this->pay_obstacle_costs){
		return 0.0;
	}
	if(this->cells.at<short>(p) == 255){
		return INFINITY;
	}
	else{
		return double(this->cells.at<short>(p));
	}
}

void Costmap_Utils::display_costmap(){ // show nice display plot and number it
	cv::namedWindow("Costmap_Utils::Cells Mat", cv::WINDOW_NORMAL);
	cv::imshow("Costmap_Utils::Cells Mat", this->displayPlot);
	cv::waitKey(0);
}

void Costmap_Utils::build_display_plot(){
	this->displayPlot= cv::Mat::zeros(cells.size(), CV_8UC3);
	for(int i=0; i<this->cells.cols; i++){
		for(int j=0; j<this->cells.rows; j++){
			cv::Point a(i,j);
			
			// annoying but it works to seed plot colors
			int c = 255 - this->cells.at<short>(a);
			cv::Vec3b ca(c,c,c);
			this->displayPlot.at<cv::Vec3b>(a) = ca;
		}
	}
}

void Costmap_Utils::add_agent_to_costmap_plot(const cv::Scalar &color, const std::vector<cv::Point> &path, const cv::Point &cLoc){
	int agent_r = round(0.025*std::min(this->cells.cols, this->cells.rows));
	int path_r = round(0.01*std::min(this->cells.cols, this->cells.rows));
	circle(this->displayPlot, cLoc, agent_r, color, -1);
	for(size_t i=1; i<path.size(); i++){

		circle(this->displayPlot, path[i], path_r, color, -1);
	}
}

bool pointCompare(const cv::Point &a, const cv::Point &b){
	if(abs(a.x - b.x) < 1.0 && abs(a.y - b.y) < 1.0){
		return true;
	}
	else{
		return false;
	}
}

bool pointOnMat(const cv::Point &a, const cv::Mat &b){
	if(a.x >= 0 && a.x < b.cols && a.y >= 0 && a.y < b.rows){
		return true;
	}
	else{
		return false;
	}
}

std::vector<cv::Point> get_image_points_at_intensity(const cv::Mat &image, const int &intensity){
	std::vector<cv::Point> temp;
	for(int i=0; i<image.cols; i++){
		for(int j=0; j<image.rows; j++){
			if(image.at<uchar>(i,j,0) == intensity){
				cv::Point t(i,j);
				temp.push_back(t);
			}
		}
	}
	return temp;
}

double lin_interp(const double &p_min, const double &p_max, const double &p){
	return (p-p_min)/(p_max-p_min);
}

void Costmap_Utils::global_to_local(const double &lat, const double &lon, double &x, double &y){
	double b = this->get_global_heading(this->origin_lat, this->origin_lon, lat, lon);
	double d = this->get_global_distance(this->origin_lat, this->origin_lon, lat, lon);
	x = d*sin(b);
	y = d*cos(b);
}


double Costmap_Utils::get_global_distance(const double &lata, const double &lona, const double &latb, const double &lonb){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(lata);
	double lon1 = this->to_radians(lona);
	double lat2 = this->to_radians(latb);
	double lon2 = this->to_radians(lonb);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double Costmap_Utils::get_global_heading(const double &lata, const double &lona, const double &latb, const double &lonb){
	double lat1 = this->to_radians(lata);
	double lat2 = this->to_radians(latb);

	double dLong = this->to_radians(lonb - lona);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double Costmap_Utils::to_radians(const double &deg){
	return deg*0.017453292519943;
}

double Costmap_Utils::to_degrees(const double&rad){
	return rad*57.295779513082;
}

bool Costmap_Utils::point_in_cells(const cv::Point &p){
	//ROS_WARN("p: %i, %i", p.x, p.y);
	//ROS_WARN("ms: %i, %i", this->map_width_cells, this->map_height_cells);
	if(p.x >= 0 && p.y >= 0){
		if(p.x < this->map_width_cells && p.y < this->map_height_cells){
			return true;
		}
	}
	return false;
}

/*
Point2d Costmap_Utils::local_to_global(const Point2d &local){
	Point2d global;

	double C_EARTH = 6378136.6;
	double dlati = local.x / C_EARTH;
	double lati = dlati + this->origin_lati;
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]


	return global;
}

Point2d Costmap_Utils::global_to_local(const Point2d &global){
	Point2d local;
	double d = distance_from_a_to_b( home_lati, origin_longti, lati, longti )
	double b = heading_from_a_to_b( home_lati, origin_longti, lati, longti )

	local.x = -d*math.sin(b)
	local.y = d*math.cos(b)

	return local;
}
*/

