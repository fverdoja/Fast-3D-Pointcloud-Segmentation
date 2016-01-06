/*
 * Object.cpp
 *
 *  Created on: 05 gen 2016
 *      Author: alex
 */

#include "Object.h"

Object::Object() {
	color.r = roundf((((float)rand() / RAND_MAX))*100)/100;
	color.g = roundf((((float)rand() / RAND_MAX))*100)/100;
	color.b = roundf((((float)rand() / RAND_MAX))*100)/100;
}

Object::~Object(){
	cout << "\n -> OGGETTO DISTRUTTO \n";
}

ObjectColor Object::get_color() {
	return color;
}

std::map<uint32_t, Supervoxel<PointT>::Ptr> Object::get_supervoxel_set(){
	return supervoxel_set;
}

std::pair<uint32_t, Supervoxel<PointT>::Ptr> Object::get_supervoxel(
		uint32_t supervoxel_label) {
	std::map<uint32_t, Supervoxel<PointT>::Ptr>::iterator itr =  supervoxel_set.find(supervoxel_label);
	return make_pair(itr->first, itr->second);
}

void Object::add_supervoxel(std::pair<uint32_t, Supervoxel<PointT>::Ptr> s) {
	supervoxel_set.insert(s);
}

bool Object::remove_supervoxel(uint32_t s) {
	if(supervoxel_set.erase(s) == 1)
		return true;
	return false;
}

bool Object::find_supervoxel(uint32_t s) {
	if(supervoxel_set.find(s) != supervoxel_set.end())
		return true;
	return false;
}

void Object::print(){
	cout << "\nObject structure:\n";
	cout << "\tcolor: r:" << color.r << ", g:" << color.g << ", b:" << color.b << "\n";
	cout << "\tsupervoxels:\n\t\t";

	map<uint32_t, Supervoxel<PointT>::Ptr>::iterator itr = supervoxel_set.begin();
	for(int i = 0; itr != supervoxel_set.end(); itr++, i++){
		cout << itr->first << "\t";
		if(i>10){
			std::cout << "\n\t\t";
			i = 0;
		}

	}
}
