/*
 * Costmap.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Sorted_list_H_
#define SRC_Sorted_list_H_

// normal c++ libs
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>
#include <math.h>

using namespace std;
using namespace cv;

class Sorted_list{

public:
	Sorted_list();
	~Sorted_list();

	void add_item(const int &index, const double &score);
	void remove_item(const int &index, const double &score);
	bool binary_search(const int &index, const double &score, int &mid);
	void print_list();

	vector<int> i_list; // keys
	vector<double> s_list; // scores, sorted low->high
};

#endif /* SRC_Sorted_list_H_ */
