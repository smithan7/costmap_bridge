/*
 * Costmap.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Sorted_list.h"

using namespace std;
using namespace cv;

Sorted_list::Sorted_list(){}

Costmap::~Costmap(){}

void Sorted_list::add_item(const int &index, const double &score ){
	int i+1 = this->binary_search(index, score);
	this->i_list.insert(this->i_list.begin() + i, index);
	this->s_list.insert(this->s_list.begin() + i, score);
}

void Sorted_list::remove_item(const int &index, const double &score){
	if(score >= 0){ // check if its viable, looking by value is faster than by index
		int i = this->binary_search(index, score);
		this->i_list.delete(this->i_list.begin() + i);
		this->s_list.delete(this->s_list.begin() + i);
	}
	else{ // linear search through list
		for(size_t i=0; i<this->i_list.size(); i++){
			if(this->i_list[i] == index){
				this->i_list.delete(this->i_list.begin() + i);
				this->s_list.delete(this->s_list.begin() + i);
			}
		}
	}
}

bool Sorted_list::binary_search(const int &index, const double &score, int &mid){

	int low, high;

	low = 0;
	high = this->s_list.size()-1;
	while( high - low > 1 ) {
		mid = (low + high) / 2;
		if(this->s_list[mid] > score) {
			high = mid - 1;
		}
		else if(this->s_list[mid] < score) {
			low = mid + 1;
		}
		else if(this->s_list[mid] == score){
			// check if index aligns
			if(this->i_list[mid] == index){
				return true;
			}
			// check below
			int t_mid = mid-1;
			while(this->s_list[t_mid]==score){
				if(this->i_list[mid] == index){
					mid = t_mid;
					return true;
				}
				t_mid--;
			}
			// check above
			int t_mid = mid+1;
			while(this->s_list[t_mid] == score){
				if(this->i_list[mid] == index){
					mid = t_mid;
					return true;
				}
				t_mid++;
			}
		}
	}
}

void Sorted_list::print_list(){
	printf("Sorted_list\n");
	for(int i=0; i<this->i_list.size(); i++){
		printf("     %i : %0.2f \n", index, score);
	}
}

