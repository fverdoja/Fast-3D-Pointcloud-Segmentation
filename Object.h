/*
 * Object.h
 *
 *  Created on: 05 gen 2016
 *      Author: alex
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <cstdlib>
#include <iostream>

using namespace pcl;
using namespace std;

typedef PointXYZRGBA PointT;

struct ObjectColor {
		float r;
		float g;
		float b;
};

class Object {

	ObjectColor color;


	public:
		std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_set;
		Object();
		~Object();
		ObjectColor get_color();
		std::map<uint32_t, Supervoxel<PointT>::Ptr> get_supervoxel_set();
		std::pair<uint32_t, Supervoxel<PointT>::Ptr> get_supervoxel(
				uint32_t supervoxel_label);
		void add_supervoxel(std::pair<uint32_t, Supervoxel<PointT>::Ptr> s);
		bool remove_supervoxel(uint32_t s);
		bool find_supervoxel(uint32_t s);
		void print();
};

#endif /* OBJECT_H_ */
