/*
 * Costmap.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef STATE_H_
#define STATE_H_

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

class State {
public:
	State(const double &xi, const double &yi, const double &si, const double &ti){this->x=xi; this->y=yi; this->speed=si; this->theta=ti; this->vx=0.0; this->vy=0.0;};
	State(){this->x=0.0; this->y=0.0; this->speed=0.0; this->theta=0.0; this->vx=0.0; this->vy=0.0;};
	~State(){};
	double get_x() {return this->x;};
	double get_y() {return this->y;};
	double get_speed() {return this->speed;};
	double get_theta() {return this->theta;};
	double get_vx() {return this->vx;};
	double get_vy() {return this->vy;};
	
	double calc_speed(){this->speed = sqrt(pow(this->vx,2) + pow(this->vy,2));};
	double calc_theta(){this->theta = atan2(vy,vx);};
	double calc_vels(){
		this->vx = this->speed*cos(this->theta);
		this->vy=this->speed*sin(this->theta);
	}

	void restrict_vels(const double &max_speed){
		this->vx *= max_speed / this->speed;
		this->vy *= max_speed / this->speed;
	}

	double set_x(const double &xi) {this->x = xi;};
	double set_y(const double &yi) {this->y = yi;};
	double set_vx(const double &vxi) {this->vx = vxi;};
	double set_vy(const double &vyi) {this->vy = vyi;};
	double set_theta(const double &ti) {this->theta = ti;};
	double set_speed(const double &si) {this->speed = si;};

private:
	double x,y,vx,vy,speed,theta;
};

#endif /* STATE_H_ */