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
	State(const double &xi, const double &yi, const double &si, const double &ti){this->x=xi; this->y=yi; this->speed=si; this->theta=ti;};
	~State(){};
	double get_x() {return this->x;};
	double get_y() {return this->y;};
	double get_speed() {return this->speed;};
	double get_theta() {return this->theta;};

	double set_x(const double &xi) {this->x = xi;};
	double set_y(const double &yi) {this->y = yi;};
	double set_theta(const double &ti) {this->theta = ti;};
	double set_speed(const double &si) {this->speed = si;};

private:
	double x,y,speed,theta;
};

#endif /* STATE_H_ */