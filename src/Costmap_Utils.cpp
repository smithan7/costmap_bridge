/*
 * Costmap.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */




#include "Costmap_Utils.h"

bool pointCompare(const cv::Point &a, const cv::Point &b);
bool pointOnMat(const cv::Point &a, const cv::Mat &b);
std::vector<cv::Point> get_image_points_at_intensity(const cv::Mat &image, const int &intensity);
double lin_interp(const double &p_min, const double &p_max, const double &p);


Costmap_Utils::Costmap_Utils(const int &test_environment_number, const int &agent_index, const int &jetson, const int &param_seed){

	this->rand_seed = param_seed;

	// I need to be initialized
	this->need_initialization = true;
	this->map_size_meters.x = 0.0;
	this->map_size_meters.y = 0.0;

	// set heuristic for A*
	this->a_star_heuristic_weight = 3.0; // 1->inf get greedier

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
	this->pay_obstacle_costs = true;

	// annoying but it works to seed plot colors
	cv::Vec3b a(255,255,255);
	this->cObsFree = a;
	a = cv::Vec3b(200,200,200);
	this->cInfFree = a;
	a = cv::Vec3b(0,0,0);
	this->cObsOccupied = a;
	a = cv::Vec3b(50,50,50);
	this->cInfOccupied = a;

	/*
	char vert_file[200];
	sprintf(vert_file, "/home/andy/catkin_ws/src/dmcts_world/worlds/circles_64_64/blank_100_100.jpg");//circles_64_64.jpg");
	
	if(jetson == 1){
		sprintf(vert_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_vertices.xml", test_environment_number);
	}
	else{
		sprintf(vert_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_vertices.xml", test_environment_number);
	}

    cv::FileStorage f_verts(vert_file, cv::FileStorage::READ);
    if (!f_verts.isOpened()){
        ROS_ERROR("Costmap_Bridge::Costmap_Utils::init::Failed to open %s", vert_file);
        return;
    }
	ROS_INFO("Costmap_Bridge::Costmap_Utils::init::Opened: %s", vert_file);
    
	std::vector<double> corners;
	f_verts["corners"] >> corners;
	this->NW_Corner.x = corners[0];
	this->NW_Corner.y = corners[1];
	this->SE_Corner.x = corners[2];
	this->SE_Corner.y = corners[3];
	this->SW_Corner.x = corners[0];
	this->SW_Corner.y = corners[3];
	this->NE_Corner.x = corners[2];
	this->NE_Corner.y = corners[1];

	this->map_size_meters = this->global_to_local(this->NE_Corner);
	this->map_size_meters.x = abs(this->map_size_meters.x);
	this->map_size_meters.y = abs(this->map_size_meters.y);
	
	ROS_INFO("    map size: %0.2f, %0.2f (meters)", this->map_size_meters.x, this->map_size_meters.y);

	// set cells per meter
	this->meters_per_cell = 1.0;

	// set meters per cell
	this->cells_per_meter = 1.0;;

	ROS_INFO("    cells per meter: %0.2f", this->cells_per_meter);
	this->map_size_cells.x = ceil(this->map_size_meters.x * this->cells_per_meter);
	this->map_size_cells.y = ceil(this->map_size_meters.y * this->cells_per_meter);
	ROS_INFO("    cells size: %i, %i", this->map_size_cells.x, this->map_size_cells.y);

	// initialize cells
	cv::Mat b = cv::Mat::ones( this->map_size_cells.y, this->map_size_cells.x, CV_16S)*this->infFree;
	this->cells = b.clone();
	ROS_INFO("    map size: %i, %i (cells)", this->cells.cols, this->cells.rows);
	*/

	this->map_size_cells = cv::Point(100, 100);
	this->map_size_meters = cv::Point2d(100.0, 100.0);

	// create obs mat for trials
	srand(this->rand_seed);
	this->create_obs_mat();

	// seed into cells satelite information
	this->seed_img();


	// initialize display plot
	this->build_cells_plot();
	//this->display_costmap();// show nice display plot and number it

	// announce I am initialized!
	this->need_initialization = false;
	ROS_INFO("Costmap_Utils::initialize_costmap::complete");

	/*
	char agent_file[200];
	if(jetson == 1){
		sprintf(agent_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/agent%i_params.xml", agent_index);
	}
	else{
    	sprintf(agent_file, "/home/andy/catkin_ws/src/distributed_planner/params/agent%i_params.xml", agent_index);
	}
    cv::FileStorage f_agent(agent_file, cv::FileStorage::READ);
    if (!f_agent.isOpened()){
        ROS_ERROR("Costmap_Bridge::Costmap_Utils::init::Failed to open %s", agent_file);
        return;
    }
	ROS_INFO("Costmap_Bridge::Costmap_Utils::init::Opened: %s", agent_file);
    

	int pay_obstacle_costs = 0;
	f_agent["pay_obstacle_cost"] >> this->pay_obstacle_costs;
	f_agent.release();
	
	if(pay_obstacle_costs == 1){
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.05;
		this->infOcc_cost = 1.0;
		this->obsOcc_cost = 10.0;
	}
	else{
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.0;
		this->infOcc_cost = 0.0;
		this->obsOcc_cost = 0.0;
	}
	*/

	cv::Point s(25,25);
	cv::Point g(35,35);
	//this->test_a_star_planner(s,g);

	State s_state(5.0,5.0,0.0,0.785398);
	State g_state(40.0,40.0,0.0,0.0);
	//this->test_kinematic_a_star_planner(s_state, g_state);
}

double Costmap_Utils::rand_double_in_range(const double &min, const double &max) {
	// get random double between min and max
	return (max - min) * double(rand()) / double(RAND_MAX) + min;
}

void Costmap_Utils::create_obs_mat(){
	std::vector<cv::Point2d> starting_locs;
	starting_locs.push_back(cv::Point2d(25,25));
	starting_locs.push_back(cv::Point2d(75,75));
	starting_locs.push_back(cv::Point2d(25,75));
	starting_locs.push_back(cv::Point2d(75,25));
	starting_locs.push_back(cv::Point2d(50,75));
	starting_locs.push_back(cv::Point2d(75,50));
	starting_locs.push_back(cv::Point2d(50,25));
	starting_locs.push_back(cv::Point2d(25,50));

	this->Obs_Mat = cv::Mat::zeros(this->map_size_cells.x, this->map_size_cells.y, CV_8UC1);
	this->obstacles.clear();
	ROS_INFO("DMCTS::World::make_obs_mat: making obstacles");
	while(this->obstacles.size() < 10){
		//ROS_INFO("making obstacle");
		// create a potnetial obstacle
		double rr = rand_double_in_range(1,10);
		double xx = rand_double_in_range(0,this->map_size_meters.x);
		double yy = rand_double_in_range(0,this->map_size_meters.y);
		//ROS_INFO("obs: %.1f, %.1f, r =  %.1f", xx, yy, rr);
		// check if any starting locations are in an obstacle
		bool flag = true;
		for(size_t s=0; s<starting_locs.size(); s++){
			double d = sqrt(pow(xx-starting_locs[s].x,2) + pow(yy-starting_locs[s].y,2));
			//ROS_INFO("starting_locs: %.1f, %.1f, d = %.1f", starting_locs[s].x, starting_locs[s].y, d);
			if(rr+2 >= d ){
				// starting loc is in obstacle
				flag = false;
				break;
			}
		}

		if(flag){
			for(size_t s=0; s<this->obstacles.size(); s++){
				double d = sqrt(pow(xx-this->obstacles[s][0],2) + pow(yy-this->obstacles[s][1],2));
				//ROS_INFO("starting_locs: %.1f, %.1f, d = %.1f", starting_locs[s].x, starting_locs[s].y, d);
				if(rr+1 >= d || this->obstacles[s][2]+1 >= d){
					// obstacle is in obstacle
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

	for(int j=4; j>0; j--){
		for(size_t i=0; i<obstacles.size(); i++){
			cv::circle(this->Obs_Mat, cv::Point(this->obstacles[i][0], this->obstacles[i][1]), this->obstacles[i][2]+j, cv::Scalar(255 - 25*j), -1);	
		}
	}

	//cv::namedWindow("Costmap_Utils::Obstacles", cv::WINDOW_NORMAL);
	//cv::imshow("Costmap_Utils::Obstacles", this->Obs_Mat);
	//cv::waitKey(0);
}

void Costmap_Utils::test_a_star_planner(const cv::Point &s, const cv::Point &g){
	cv::Mat tst = this->displayPlot.clone();
	
	cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
	cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

	std::vector<cv::Point> path; 
	double length = 0.0;
	this->a_star_path(s,g, path, length);

	/*
	for(size_t i=0; i<path.size(); i++){
		cv::circle(tst, path[i], 1, cv::Scalar(255,0,0), -1);		
	}

	cv::namedWindow("a_star_path", CV_WINDOW_NORMAL);
	cv::imshow("a_star_path", tst);
	cv::waitKey(0);
	*/
}


void Costmap_Utils::test_kinematic_a_star_planner(State &s_state, State &g_state){
	cv::Mat tst = this->displayPlot.clone();
	cv::Point s,g;
	this->local_to_cells(cv::Point2d(s_state.get_x(),s_state.get_y()), s);
	this->local_to_cells(cv::Point2d(g_state.get_x(),g_state.get_y()), g);
	cv::circle(tst, s, 2, cv::Scalar(0,180,0), -1);
	cv::circle(tst, g, 2, cv::Scalar(0,0,180), -1);

	std::vector<State> path; 
	double length_time = 0.0;
	this->a_star_kinematic(s_state,g_state, path, length_time);

	for(size_t i=0; i<path.size(); i++){
		cv::Point t;
		this->local_to_cells(cv::Point2d(path[i].get_x(), path[i].get_y()), t);
		cv::circle(tst, t, 1, cv::Scalar(255,0,0), -1);		
	}

	cv::namedWindow("a_star_kinematic", CV_WINDOW_NORMAL);
	cv::imshow("a_star_kinematic", tst);
	cv::waitKey(0);
}


Costmap_Utils::~Costmap_Utils() {}

void Costmap_Utils::seed_img(){
	//cv::Mat seed = cv::imread("/home/andy/catkin_ws/src/dmcts_world/worlds/circles_64_64/blank_100_100.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat seed = cv::imread("/home/andy/catkin_ws/src/dmcts_world/worlds/circles_64_64/walls_100_100.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat seed = cv::imread("/home/andy/catkin_ws/src/dmcts_world/worlds/circles_64_64/circles_64_64.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	
	cv::Mat seed = this->Obs_Mat.clone();

	if(!seed.data && !this->Obs_Mat.data){
		if(!seed.data){
			ROS_ERROR("Costmap::seed_img::Could NOT load img");
		}
		else{
			ROS_ERROR("Costmap::this->Obs_Mat is empty");	
		}
		return;
	}

	//cv::namedWindow("seed", CV_WINDOW_NORMAL);
	//cv::imshow("seed", seed);
	//cv::waitKey(10);
	
	this->cells_per_meter = 1.0;
	this->meters_per_cell = 1.0 / this->cells_per_meter;

	this->map_size_cells = cv::Point(seed.cols, seed.rows);
	this->map_size_meters = cv::Point2d(seed.cols * this->meters_per_cell, seed.rows * this->meters_per_cell);	

	cv::Mat b = cv::Mat::ones( this->map_size_cells.y, this->map_size_cells.x, CV_16S)*this->infFree;
	this->cells = b.clone();

	cv::Mat seed_b;
	for(int i=0; i<3; i++){
		blur(seed, seed_b, cv::Size(5,5));
		cv::min(seed, seed_b, seed);
	}

	//cv::namedWindow("seed_b", CV_WINDOW_NORMAL);
	//cv::imshow("seed_b", seed_b);
	//cv::waitKey(10);

	//cv::namedWindow("seed2", CV_WINDOW_NORMAL);
	//cv::imshow("seed2", seed);
	//cv::waitKey(0);

	cv::resize(seed, seed, this->cells.size());
	
	// go through every pixel of the image and occupancy map
	for(int i=0; i<seed.cols; i++){
		for(int j=0; j<seed.rows; j++){
			cv::Point p(i,j);
			if(seed.at<uchar>(p) == 255){
				this->cells.at<short>(p) = 255;
			}
			else{
				this->cells.at<short>(p) = int(100.0 * (double(seed.at<uchar>(p))/255.0));
				//std::cout << this->cells.at<short>(p) << ", ";
			}
			/*
			if(seed.at<uchar>(p) >= 200 ){
				this->cells.at<short>(p) = this->obsOccupied;
			}
			else if(seed.at<uchar>(p) >= 50 ){
				this->cells.at<short>(p) = this->infOccupied;
			}
			else{
				this->cells.at<short>(p) = this->obsFree;
			}
			*/
		}
		//std::cout << std::endl;
	}
	
	//ROS_INFO("seeded cells");
}

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
		ROS_INFO("cells.size(): %i, %i", this->map_size_cells.x, this->map_size_cells.y);		
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
	p.x = floor( l / this->map_size_cells.x );
	p.y = l % this->map_size_cells.y;

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
	loc.x = double(cell.x) / this->meters_per_cell;
	loc.y = double(cell.y) / this->meters_per_cell;
}


void Costmap_Utils::local_to_cells(const cv::Point2d &loc, cv::Point &cell){
	// move from local x/y meters to costmap cell
	cell.x = round(this->cells_per_meter * loc.x);
	cell.y = round(this->cells_per_meter * loc.y);
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


double Costmap_Utils::a_star_kinematic_heuristic(State &c, State &g){
	// c.x, c.y, c.theta, c.speed
	double euclid_dist = sqrt(pow(c.get_x()-g.get_x(),2) + pow(c.get_y()-g.get_y(),2));
	double g_theta = atan2(g.get_y()-c.get_y(), g.get_x()-c.get_x());
	double theta_dist = fabs(c.get_theta()-g_theta);
	double speed_cost = 0.0; // speed cost only matters if I am almost there
	if(euclid_dist < this->euclid_threshold){
		theta_dist = fabs(g.get_theta() - c.get_theta());
		speed_cost = fabs(g.get_speed() - c.get_speed());
	}

	double sum_cost = this->kinematic_weight[0] * euclid_dist / std::max(0.001, c.get_speed());
	sum_cost += this->kinematic_weight[1] * theta_dist;
	sum_cost += this->kinematic_weight[2] * speed_cost;

	/*
	std::cout << "c: " << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << std::endl;
	std::cout << "g: " << g[0] << "," << g[1] << "," << g[2] << "," << g[3] << std::endl;
	std::cout << "euclid_dist: " << euclid_dist << std::endl;
	std::cout << "theta_dist: " << theta_dist << std::endl;
	std::cout << "speed_cost: " << speed_cost << std::endl;
	std::cout << "kinematic_weight: " << this->kinematic_weight[0] << "," << this->kinematic_weight[1] << "," << this->kinematic_weight[2] << std::endl;
	std::cout << "sum_cost: " << sum_cost << std::endl;
	*/

	return sum_cost;
}

bool Costmap_Utils::a_star_kinematic(State &sState, State &gState, std::vector<State> &path, double &length_time){
	//std::cout << "in a_star_kinematic" << std::endl;
	this->kinematic_goal_tolerance = 1.0;
	this->speed_step = 0.125; // m/s
	this->ang_step =  3*0.0872222; // approx 5 degs
	this->max_speed = 2.5;
	this->kinematic_weight.clear();
	this->kinematic_weight.push_back(1.0); // euclid
	this->kinematic_weight.push_back(0.0); // bearing
	this->kinematic_weight.push_back(0.0); // speed

	State s = State(1.0,1.0,10.0,10.0);

	if(this->need_initialization){
		return false;
	}
	//std::cout << "did not need_initialization" << std::endl;

	// ensure that 
	path.clear();
	length_time = 0.0;

	//std::cout << "checking if I start at goal" << std::endl;
	if(this->a_star_kinematic_heuristic(sState, gState) < this->kinematic_goal_tolerance){
		return true;
	}
	//std::cout << "did not start at goal" << std::endl;

	// initialize A* stuff
	std::vector<State> set; // all states
	set.push_back(sState);
	std::vector<int> cameFrom; // one for every state in set
	cameFrom.push_back(-1);
	std::vector<double> gScore; // known cost from start to me
	std::vector<double> fScore; // heuristic cost from me to goal

	std::vector<int> oSet; // index of states to evaluate, index corresponds to set
	
	oSet.push_back(0); // starting node in open set
	gScore.push_back(0.0); // starting node has score 0
	fScore.push_back(this->a_star_heuristic_weight * this->a_star_kinematic_heuristic(sState, gState));

	std::vector<double> n_s = {0.0, this->speed_step, -this->speed_step, 0.0, 0.0}; // neighbor speed changes
	std::vector<double> n_t = {0.0, 0.0, 0.0, this->ang_step, -this->ang_step}; // neighbor angle changes
	//std::cout << "going into while loop" << std::endl;
	int loop_cntr = 0;
	while(oSet.size() > 0){
		loop_cntr++;
		//std::cout << "start of while loop" << std::endl;
		// find node with lowest fScore and make current
		double min = INFINITY;
		int mindex = -1;
		int oSet_i = -1;
		for(size_t i=0; i<oSet.size(); i++){
		//	std::cout << "loop_cntr: " << loop_cntr << " i: " << i << " oSet[i]: " << oSet[i] << std::endl;
		//	std::cout << "loop_cntr: " << loop_cntr << " i: " << i << " oSet[i]: " << oSet[i] << " and fScore: " << fScore[oSet[i]] << " and state: " << set[oSet[i]][0] << " , " << set[oSet[i]][1] << " , " << set[oSet[i]][2] << " , " << set[oSet[i]][3] << std::endl;
			if(fScore[oSet[i]] < min){
				min = fScore[oSet[i]];
				mindex = oSet[i];
				oSet_i = i;
			}
		}

		if( mindex < 0){
			// I did NOT find mindex! Search over!
			path.clear();
			length_time = INFINITY;
			return false;
		}
		
		// I did find mindex, do maintenance
		State c_state = set[mindex]; // set current location
		oSet.erase(oSet.begin() + oSet_i); // erase from open set
		/*
		std::cout << "found a mindex: " << mindex << " of set.size(): " << set.size() << std::endl;
		for(size_t i=0; i<set.size(); i++){
			std::cout << "set[" << i << "]: " << set[i][0] << ", " << set[i][1] << ", " << set[i][2] << ", " << set[i][3] << " and fScore: " << fScore[i] << " and gScore: " << gScore[i] << " and cameFrom: " << cameFrom[i] << std::endl;
		}
		*/

		//std::cout << "mindex[" << loop_cntr << "]: " << mindex << " has state: " << set[mindex].get_x() << " , " << set[mindex].get_y() << " , " << set[mindex].get_speed() << " , " << set[mindex].get_theta() << " and a_star_kinematic_heuristic: " << this->a_star_kinematic_heuristic(set[mindex], gState) << std::endl;
		

		//std::cout << "set.size(): " << set.size() << std::endl;
		//std::cout << "fScore.size(): " << fScore.size() << std::endl;
		//std::cout << "gScore.size(): " << gScore.size() << std::endl;
		//std::cout << "cameFrom.size(): " << cameFrom.size() << std::endl;

		
		
		if(loop_cntr > 5000){
			return false;
		}
		
		//for(size_t i=0; i<oSet.size(); i++){
		//	std::cout << i << ": " << oSet[i] << std::endl;
		//}

		//std::cout << "mindex: " << mindex << std::endl;


		
		//for(size_t i=0; i<oSet.size(); i++){
		//	std::cout << i << ": " << oSet[i] << std::endl;
		//}


		//std::cout << "checking if mindex is at my goal" << std::endl;
		// am I at the goal?
		if( this->a_star_kinematic_heuristic(set[mindex], gState) < this->kinematic_goal_tolerance){
			//std::cout << " is in goal range" << std::endl;
			// I am, construct the path 
			//std::cout << "mindex: " << mindex << std::endl;
			//std::cout << "  set: " << set[mindex][0] << ", " << set[mindex][1] << ", " << set[mindex][2] << ", " << set[mindex][3] << std::endl;	
			length_time = gScore[mindex];
			path.push_back(gState);
			while( cameFrom[mindex] != 0 ){ // work backwards to start
				//std::cout << "mindex: " << mindex << std::endl;
				//std::cout << "  set: " << set[mindex][0] << ", " << set[mindex][1] << std::endl;
				path.push_back(set[mindex]); // append path
				mindex = cameFrom[mindex];
			}
			reverse(path.begin(),path.end());
			return true;
		}
		//std::cout << " is not at goal" << std::endl;

		// not at the goal, get new nbrs
		for(size_t ni = 0; ni<n_s.size(); ni++){
			// potential nbr
			//std::cout << "nbr change theta: " << n_t[ni] << " and speed " << n_s[ni] << std::endl; 
			State n_state(0.0,0.0,c_state.get_speed() + double(n_s[ni]),c_state.get_theta() + double(n_t[ni]));

			// don't use a quad that goes backwards!
			if(n_state.get_speed() < 0 || n_state.get_speed() > this->max_speed){
				//std::cout << "speed less than 0, continueing" << std::endl;
				continue;
			}
			
			n_state.set_x(c_state.get_x() + (cos(c_state.get_theta())*c_state.get_speed() + cos(n_state.get_theta())*n_state.get_speed())/2.0);
			n_state.set_y(c_state.get_y() + (sin(c_state.get_theta())*c_state.get_speed() + sin(n_state.get_theta())*n_state.get_speed())/2.0);
			
			//std::cout << "ni: " << ni << " nbr: " << n_state[0] << " , " << n_state[1] << " , " << n_state[2] << " , " << n_state[3] << std::endl;			
			double occ_pen = this->kinematic_get_occ_penalty(n_state);
			if(occ_pen < 1.0){
				//std::cout << "appending nbr" << std::endl;			
				set.push_back( n_state ); // add to list of all states
				oSet.push_back( set.size() - 1 ); // I am open, point to set index
				cameFrom.push_back( mindex ); // where did I come from
				
				// calc temporary gscore, estimate of total cost
				gScore.push_back( gScore[mindex] + (1 + occ_pen) ); // always move forward 1 step 
				fScore.push_back( gScore.back() + this->a_star_heuristic_weight * this->a_star_kinematic_heuristic(n_state, gState) );
			}

			//for(size_t s=0; s<set.size(); s++){
			//	std::cout << "set[" << s << "]: " << set[s][0] << ", " << set[s][1] << ", " << set[s][2] << ", " << set[s][3] << std::endl;
			//}
		}
		//std::cout << "checked nbrs" << std::endl;
	}
	return false;
}

double Costmap_Utils::kinematic_get_occ_penalty(State &state){
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


				if(cells.at<short>(nbr) < 127){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + this->a_star_heuristic_weight * this->get_cells_euclidian_distance(gLoc,nbr);
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
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
	
	//ROS_INFO("cells: %i", this->cells.at<short>(p));
	//ROS_INFO("cells d: %0.2f", double(this->cells.at<short>(p)));

	if(this->cells.at<short>(p) == 255){
		return INFINITY;
	}
	else{
		//if(this->cells.at<short>(p) > 0){
		//	ROS_INFO("cells: %i", this->cells.at<short>(p));
		//	ROS_INFO("cells d: %0.2f", double(this->cells.at<short>(p)));
		//}
		return double(this->cells.at<short>(p));
	}
	/*
	if(this->cells.at<short>(p) == this->obsFree){
		return this->obsFree_cost;
	}
	else if(this->cells.at<short>(p) == this->infFree){
		return this->infFree_cost;
	}
	else if(this->cells.at<short>(p) == this->infOccupied){
		return this->infOcc_cost;
	}
	else if(this->cells.at<short>(p) == this->obsOccupied){
		return this->obsOcc_cost;
	}
	else{
		// something went wrong, probably shouldn't go there...
		return double(INFINITY);
	}
	*/
}

void Costmap_Utils::display_costmap(){ // show nice display plot and number it
	cv::namedWindow("Costmap_Utils::Cells Mat", cv::WINDOW_NORMAL);
	cv::imshow("Costmap_Utils::Cells Mat", this->displayPlot);
	cv::waitKey(0);
}

void Costmap_Utils::build_cells_plot(){
	this->displayPlot= cv::Mat::zeros(cells.size(), CV_8UC3);
	for(int i=0; i<this->cells.cols; i++){
		for(int j=0; j<this->cells.rows; j++){
			cv::Point a(i,j);
			
			// annoying but it works to seed plot colors
			int c = 255 - this->cells.at<short>(a);
			cv::Vec3b ca(c,c,c);
			this->displayPlot.at<cv::Vec3b>(a) = ca;
			/*
			if(this->cells.at<short>(a) == 0){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) > 0){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsOccupied;
			}
			*/
			/*
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->infFree){
				this->displayPlot.at<cv::Vec3b>(a) = this->cInfFree;
			}
			else if(this->cells.at<short>(a) == this->obsOccupied){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsOccupied;
			}
			else if(this->cells.at<short>(a) == this->infOccupied){
				this->displayPlot.at<cv::Vec3b>(a) = this->cInfOccupied;
			}
			*/
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

cv::Point2d Costmap_Utils::global_to_local(const cv::Point2d &loc){
	double b = this->get_global_heading(this->SW_Corner, loc);
	double d = this->get_global_distance(this->SW_Corner, loc);
	//ROS_INFO("Distance/Bearing: %0.2f, %0.2f", d,b);
	//ROS_INFO("SW_Corner: %0.6f, %0.6f", this->SW_Corner.x, this->SW_Corner.y);
	cv::Point2d l;
	l.x = d*sin(b);
	l.y =this->map_size_meters.y - d*cos(b);
	//ROS_INFO("Point: %0.2f, %0.2f", d*sin(b), d*cos(b));
	//ROS_INFO("Map size: %0.2f, %0.2f", this->map_size_meters.x, this->map_size_meters.y);
	return l;
}


double Costmap_Utils::get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(g1.y);
	double lon1 = this->to_radians(g1.x);
	double lat2 = this->to_radians(g2.y);
	double lon2 = this->to_radians(g2.x);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double Costmap_Utils::get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2){
	double lat1 = this->to_radians(g1.y);
	double lat2 = this->to_radians(g2.y);

	double dLong = this->to_radians(g2.x - g1.x);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double Costmap_Utils::to_radians(const double &deg){
	return deg*3.141592653589 / 180.0;
}

bool Costmap_Utils::point_in_cells(const cv::Point &p){
	//ROS_WARN("p: %i, %i", p.x, p.y);
	//ROS_WARN("ms: %i, %i", this->map_size_cells.x, this->map_size_cells.y);
	if(p.x >= 0 && p.y >= 0){
		if(p.x < this->map_size_cells.x && p.y < this->map_size_cells.y){
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

