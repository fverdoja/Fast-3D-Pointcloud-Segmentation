/*
 * Clustering.cpp
 *
 *  Created on: 19/05/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 *
 *
 * Software License Agreement (BSD License)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "Clustering.h"

// ALEX CODE

void Clustering::addSupervoxelToObject(uint32_t obj_number,
		pair<uint32_t, Supervoxel<PointT>::Ptr> supervoxel,
		map<uint32_t, Object*> &objects_set) {

	Object* tmpObj = new Object();

	// if it's a new object, create it
	if(objects_set.find(obj_number) == objects_set.end())
		objects_set.insert(make_pair(obj_number, tmpObj));

	tmpObj = objects_set.find(obj_number)->second;
	tmpObj->add_supervoxel(supervoxel);

}

int Clustering::findSupervoxelFromObject(uint32_t obj_number,
		uint32_t supervoxel_label, map<uint32_t, Object*> objects_set) {

	map<uint32_t, Object*>::iterator itr = objects_set.find(obj_number);

	// object <obj_number> doesn't exists
	if (itr == objects_set.end())
		return -1;
	// object <obj_number> effectively contains supervoxel_label
	else if (itr->second->find_supervoxel(supervoxel_label))
		return 1;
	// object <obj_number> doesn't contain supervoxel_label
	return 0;
}

int Clustering::removeSupervoxelFromObject(uint32_t obj_number,
		uint32_t supervoxel_label, map<uint32_t, Object*> &objects_set) {
	int found = Clustering::findSupervoxelFromObject(obj_number,
			supervoxel_label, objects_set);
	if (found > 0)
		objects_set.find(obj_number)->second->remove_supervoxel(
				supervoxel_label);
	return found;

}

bool Clustering::moveSupervoxelFromToObject(uint32_t obj_from, uint32_t obj_to,
		uint32_t supervoxel_label, map<uint32_t, Object*> &objects_set) {
	// Expect return 1, we expect to find obj_from and supervoxel_label into
	int found_from = Clustering::findSupervoxelFromObject(obj_from,
			supervoxel_label, objects_set);
	if (found_from == 1){
		Object* tmpObj = objects_set.find(obj_from)->second;
		pair<uint32_t, Supervoxel<PointT>::Ptr> tmpSupervoxel =
				tmpObj->get_supervoxel(supervoxel_label);
		tmpObj->remove_supervoxel(supervoxel_label);

		Clustering::addSupervoxelToObject(obj_to, tmpSupervoxel, objects_set);
		return true;
	}
	return false;
}

int Clustering::getObjectFromSupervoxelLabel(uint32_t supervoxel_label,
			map<uint32_t, Object*> objects_set){
	int obj_number = 1;
	int result;

	std::map<uint32_t, Object*>::iterator itr = objects_set.begin();
	for(; itr != objects_set.end(); itr++){
		if(itr->second->find_supervoxel(supervoxel_label))
			return obj_number;
		obj_number++;
	}

	return -1;
}

void Clustering::computeDisconnectedGraphs(int obj_index,
		multimap<uint32_t, uint32_t> adjacency,
		map<uint32_t, Object*> &objects_set) {

	pair<uint32_t, Object*> currentObj = make_pair(obj_index,
			objects_set.at(obj_index));

	map<uint32_t, Supervoxel<PointT>::Ptr> current_supervoxel_set =
			currentObj.second->get_supervoxel_set();

	map<uint32_t, Supervoxel<PointT>::Ptr>::iterator supervoxel_set_itr =
			current_supervoxel_set.begin();

	// For each supervoxel into current object
	while (supervoxel_set_itr != current_supervoxel_set.end()) {
		uint32_t current_supervoxel_label = supervoxel_set_itr->first;

		// Create a set with the current supervoxel and each its adjacencies
		list<uint32_t> together;

		together.push_back(current_supervoxel_label);

		Clustering::computeAdjacencies(together, adjacency,
				current_supervoxel_set, objects_set);

		if (together.size() < current_supervoxel_set.size()) {

			int obj_to = objects_set.size() + 1;

			list<uint32_t>::iterator tmpItr = together.begin();

			tmpItr = together.begin();
			for (; tmpItr != together.end(); tmpItr++) {
				Clustering::moveSupervoxelFromToObject(obj_index, obj_to,
						*tmpItr, objects_set);

			}

			current_supervoxel_set = currentObj.second->get_supervoxel_set();
			supervoxel_set_itr = current_supervoxel_set.begin();
		}

		supervoxel_set_itr++;
	}

}


void Clustering::computeAdjacencies(list<uint32_t> &together,
		multimap<uint32_t, uint32_t> adjacency,
		map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_set,
		map<uint32_t, Object*> objects_set) {

	list<uint32_t>::iterator itr = together.begin();

	for (; itr != together.end(); itr++) {
		multimap<uint32_t, uint32_t>::iterator adj_itr = adjacency.equal_range(
				*itr).first;

		for (; adj_itr != adjacency.equal_range(*itr).second; adj_itr++) {
			if (supervoxel_set.find(adj_itr->second) != supervoxel_set.end()) {
				if (find(together.begin(), together.end(), adj_itr->second)
						== together.end()) {
					together.push_back(adj_itr->second);
				}
			}
		}
	}
}

map<uint32_t, Supervoxel<PointT>::Ptr> Clustering::getGraphSupervoxels(
		multimap<uint32_t, uint32_t> adjacency,
		map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_set,
		map<uint32_t, Supervoxel<PointT>::Ptr> &graph_supervoxel) {

	multimap<uint32_t, uint32_t>::iterator itr = adjacency.begin();


	for (; itr != adjacency.end(); itr++) {
		if(graph_supervoxel.find(itr->first) == graph_supervoxel.end())
			graph_supervoxel.insert(make_pair(itr->first, supervoxel_set.find(itr->first)->second));

		multimap<uint32_t, uint32_t>::iterator adj_itr = adjacency.equal_range(
			itr->first).first;
		set<uint32_t> tmp;

		for (; adj_itr != adjacency.equal_range(itr->first).second; adj_itr++) {
			if (supervoxel_set.find(adj_itr->second) != supervoxel_set.end())
				tmp.insert(adj_itr->second);

		}

		for(set<uint32_t>::iterator it = tmp.begin(); it != tmp.end(); it++){
			if(graph_supervoxel.find(*it) == graph_supervoxel.end())
				graph_supervoxel.insert(make_pair(*it, supervoxel_set.find(*it)->second));
		}
	}
	return graph_supervoxel;
}

void Clustering::cutAdjacencies(uint32_t label, multimap<uint32_t, uint32_t>& adjacency) {
	multimap<uint32_t, uint32_t>::iterator it = adjacency.begin();

	adjacency.erase(label);

	for(; it != adjacency.end(); it++){
		if(it->second == label)
			adjacency.erase(it);
	}
}

void Clustering::cutAdjacencies(uint32_t label, float max_distance,
		set<uint32_t>& visited, multimap<uint32_t, uint32_t>& adjacency,
		map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_set) {

	multimap<uint32_t, uint32_t>::iterator it = adjacency.begin();
	visited.insert(label);

	for(; it != adjacency.end(); it++){

		if(it->first == label || it->second == label){

			uint32_t other_label = it->first == label ? it->second : it->first;

			if (supervoxel_set.find(other_label) != supervoxel_set.end() &&
					visited.find(other_label) == visited.end()) {

				PointXYZRGBA label_supervoxel =
						supervoxel_set.find(label)->second->centroid_;

				PointXYZRGBA other_supervoxel =
						supervoxel_set.find(other_label)->second->centroid_;

				float distance = sqrt(
						pow((label_supervoxel.x - other_supervoxel.x), 2.0)
								+ pow((label_supervoxel.y - other_supervoxel.y),
										2.0)
								+ pow((label_supervoxel.z - other_supervoxel.z),
										2.0));

				if (distance < max_distance)
					cutAdjacencies(other_label, max_distance, visited,
							adjacency, supervoxel_set);
				else
					adjacency.erase(it);
			}
		}
	}
}

void Clustering::edgeCutter(multimap<uint32_t, uint32_t>& adjacency,
		 map<uint32_t, Object*> objects_set, float toll_multiplier) {
	//float toll_multiplier = 1.5;
	list<edge> edge_list;
	list<edge>::iterator edge_itr = edge_list.begin();
	map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_set = objects_set.at(2)->get_supervoxel_set();

	edge_itr++;


	multimap<uint32_t, uint32_t>::iterator adj_itr = adjacency.begin();
	for (; adj_itr != adjacency.end(); adj_itr++) {
		edge tmp;
		tmp.node_a = adj_itr->first;
		tmp.node_b = adj_itr->second;
		map<uint32_t, Supervoxel<PointT>::Ptr>::iterator it_a =
				supervoxel_set.find(adj_itr->first);
		map<uint32_t, Supervoxel<PointT>::Ptr>::iterator it_b =
				supervoxel_set.find(adj_itr->second);
		if (it_a != supervoxel_set.end() && it_b != supervoxel_set.end()) {
			tmp.distance = sqrt(
					pow((it_a->second->centroid_.x - it_b->second->centroid_.x),
							2.0)
							+ pow(
									(it_a->second->centroid_.y
											- it_b->second->centroid_.y), 2.0)
							+ pow(
									(it_a->second->centroid_.z
											- it_b->second->centroid_.z), 2.0));
			edge_list.insert(edge_itr, tmp);
		}
	}

	edge_list.sort(Clustering::compare_edge);

	//cout << "Edges:";
	for (edge_itr = edge_list.begin(); edge_itr != edge_list.end();
			edge_itr++) {
		//cout << "\n[" << edge_itr->distance << "] " << edge_itr->node_a << " - "
		//		<< edge_itr->node_b;

		set<uint32_t> visited;
		float max_distance = edge_itr->distance * toll_multiplier;
		Clustering::cutAdjacencies(edge_itr->node_a, max_distance, visited,
				adjacency, supervoxel_set);
		Clustering::cutAdjacencies(edge_itr->node_b, max_distance, visited,
						adjacency, supervoxel_set);

	}

}


bool Clustering::compare_edge (const edge& first, const edge& second){
	return first.distance < second.distance;
}

void Clustering::mergeSupervoxel(std::pair<uint32_t, uint32_t> supvox_ids) {
	SupervoxelT::Ptr sup1 = state.segments.at(supvox_ids.first);
	SupervoxelT::Ptr sup2 = state.segments.at(supvox_ids.second);
	SupervoxelT::Ptr sup_new = boost::make_shared<SupervoxelT>();

	*(sup_new->voxels_) = *(sup1->voxels_) + *(sup2->voxels_);
	*(sup_new->normals_) = *(sup1->normals_) + *(sup2->normals_);

	PointT new_centr;
	computeCentroid(*(sup_new->voxels_), new_centr);
	sup_new->centroid_ = new_centr;

	Eigen::Vector4f new_norm;
	float new_curv;
	computePointNormal(*(sup_new->voxels_), new_norm, new_curv);
	flipNormalTowardsViewpoint(sup_new->centroid_, 0, 0, 0, new_norm);
	new_norm[3] = 0.0f;
	new_norm.normalize();
	sup_new->normal_.normal_x = new_norm[0];
	sup_new->normal_.normal_y = new_norm[1];
	sup_new->normal_.normal_z = new_norm[2];
	sup_new->normal_.curvature = new_curv;

	state.segments.erase(supvox_ids.first);
	state.segments.erase(supvox_ids.second);
	state.segments.insert(
			std::pair<uint32_t, SupervoxelT::Ptr>(supvox_ids.first, sup_new));
}

void Clustering::analyze_graph(Clustering& segmentation,
		multimap<uint32_t, uint32_t>& adjacency, float toll_multiplier) {
	// ALEX CODE
	// Print supervoxel label next to each centroid and
	// split table from rest of objects
	// tested on test: 50,48,46,30,25
	std::map<uint32_t, Object*> objects_set;
	int tmpIndex = 0;
	float tolerance = 0.03f;				//0.085f;
	ostringstream convert;
	Supervoxel<PointT>::Ptr referringPoint;
	uint32_t referringPoint_label;
	Eigen::Vector3f tmpVect;
	std::map<uint32_t, Supervoxel<PointT>::Ptr>::iterator supervoxel_itr,
			max_points_supervoxel;
	map<uint32_t, Supervoxel<PointT>::Ptr> graph_supervoxels, tmp_supervoxels;
	multimap<uint32_t, uint32_t>::iterator adj_itr;
	multimap<uint32_t, uint32_t> adjacency_list;
	Object* tmpObj;
	ObjectColor c;

	for(adj_itr = adjacency.begin(); adj_itr != adjacency.end(); adj_itr++){
		adjacency_list.insert(pair<uint32_t, uint32_t>(adj_itr->first, adj_itr->second));
	}

	Clustering::getGraphSupervoxels(adjacency_list, segmentation.getState().get_segments(),
			graph_supervoxels);

	// BEGIN PLANE-OBJECTS DIFFERENTIATION

	supervoxel_itr = graph_supervoxels.begin();
	max_points_supervoxel = supervoxel_itr;
	for (; supervoxel_itr != graph_supervoxels.end(); supervoxel_itr++) {
		PointCloud<PointT>::Ptr max_voxels =
				max_points_supervoxel->second->voxels_;
		PointCloud<PointT>::Ptr itr_voxels = supervoxel_itr->second->voxels_;
		if (max_voxels->size() < itr_voxels->size())
			max_points_supervoxel = supervoxel_itr;
	}
	supervoxel_itr = max_points_supervoxel;
	referringPoint_label = supervoxel_itr->first;
	referringPoint = supervoxel_itr->second;

	// NOT UNCOMMENT
	/*Clustering::addSupervoxelToObject(1,
	 make_pair(supervoxel_itr->first, supervoxel_itr->second),
	 objects_set);*/
	//viewer->addText3D(convert.str(), referringPoint->centroid_,0.01, 0.0, 0.0, 1.0);
	supervoxel_itr = graph_supervoxels.begin();

	for (; supervoxel_itr != graph_supervoxels.end();
			supervoxel_itr++, tmpIndex++) {
		uint32_t supervoxel_label = supervoxel_itr->first;
		Supervoxel<PointT>::Ptr supervoxel = supervoxel_itr->second;

		convert.str("");
		convert.clear();
		convert << "[" << supervoxel_label << "]";

		tmpVect = referringPoint->centroid_.getArray3fMap()
				- supervoxel->centroid_.getArray3fMap();
		tmpVect /= tmpVect.norm();

		Eigen::Vector3f referringPointNormal =
				referringPoint->normal_.getNormalVector3fMap();

		// Plane centroids
		if (abs(referringPointNormal.dot(tmpVect)) < tolerance) {
			Clustering::addSupervoxelToObject(1,
					make_pair(supervoxel_label, supervoxel), objects_set);
			//Clustering::cutAdjacencies(supervoxel_label, adjacency_list);
			tmpObj = (objects_set.find(1)->second);
			c = tmpObj->get_color();
		}
		// Objects supervoxels
		else {
			Clustering::addSupervoxelToObject(2,
					make_pair(supervoxel_label, supervoxel), objects_set);
			tmpObj = (objects_set.find(2)->second);
			c = tmpObj->get_color();
		}
		//viewer->addText3D(convert.str(), supervoxel->centroid_, 0.01, c.r, c.g, c.b);

	}

	Clustering::moveSupervoxelFromToObject(2, 1, referringPoint_label,
			objects_set);

	// END PLANE-OBJECTS DIFFERENTIATION

	// BEGIN Adding all connections (es. 1->71, 71->1) to adjacency_list list
	adj_itr = adjacency_list.begin();
	for (; adj_itr != adjacency_list.end(); adj_itr++) {
		std::multimap<uint32_t, uint32_t>::iterator adjacent_itr =
				adjacency_list.equal_range(adj_itr->second).first;
		bool found = false;
		for (;
				adjacent_itr != adjacency_list.equal_range(adj_itr->second).second
						&& !found; adjacent_itr++) {
			if (adjacent_itr->second == adj_itr->first)
				found = true;
		}
		if (!found)
			adjacency_list.insert(make_pair(adj_itr->second, adj_itr->first));
	}
	// END Adding all connections (es. 1->71, 71->1) to adjacency_list list

	// BEGIN Print only elements of adj_list connected to the graph
	/*adj_itr = adjacency_list.begin();
	for (; adj_itr != adjacency_list.end(); adj_itr++)
		if (Clustering::findSupervoxelFromObject(2, adj_itr->first, objects_set)
				> 0) {
			if (Clustering::findSupervoxelFromObject(2, adj_itr->second,
					objects_set) > 0) {
				cout << adj_itr->first << " -> " << adj_itr->second << "\n";
			}
		}
	*/
	// END Print only elements of adj_list connected to the graph

	// BEGIN cut plane-objects adjacencies
	tmp_supervoxels = objects_set.find(1)->second->get_supervoxel_set();
	for (supervoxel_itr = tmp_supervoxels.begin();
			supervoxel_itr != tmp_supervoxels.end(); supervoxel_itr++) {
		Clustering::cutAdjacencies(supervoxel_itr->first, adjacency_list);
	}
	// END cut plane-objects adjacencies

	// !!!
	// TODO edgeCutter on obj_index = 2 (all supervoxels except the plane)
	// !!!
	Clustering::edgeCutter(adjacency_list, objects_set, toll_multiplier);

	/*
	adj_itr = adjacency_list.begin();
	cout << "\nadjlist:";
	for (; adj_itr != adjacency_list.end(); adj_itr++) {
		cout << "\n" << adj_itr->first << " - " << adj_itr->second;
	}*/

	//cout << "\nComputing Disconnected Graphs..\n";
	Clustering::computeDisconnectedGraphs(2, adjacency_list, objects_set);

	// BEGIN print object labels clustered in objects_set
	map<uint32_t, Object*>::iterator obSetItr = objects_set.begin();
	for (; obSetItr != objects_set.end(); obSetItr++) {
		//cout << "Oggetto n. " << obSetItr->first;
		//obSetItr->second->print();
		//cout << "\n";
	}
	// END print object labels clustered in objects_set

	ClusteringT segments = segmentation.getState().get_segments();
	ClusteringT::iterator segments_it = segments.begin();

	obSetItr = objects_set.begin();
	obSetItr++;

	/*cout << "\nADJ MODIFICATA:";
	for(adj_itr = adjacency_list.begin(); adj_itr != adjacency_list.end(); adj_itr++){
		cout << "\n" << adj_itr->first << " - " << adj_itr->second;
	}

	cout << "\nADJ REALE :";
	for(adj_itr = adjacency.begin(); adj_itr != adjacency.end(); adj_itr++){
		cout << "\n" << adj_itr->first << " - " << adj_itr->second;
	}*/

	for (; obSetItr != objects_set.end(); obSetItr++) {
		tmp_supervoxels = obSetItr->second->get_supervoxel_set();

		if(tmp_supervoxels.size() > 1){
			supervoxel_itr = tmp_supervoxels.begin();
			supervoxel_itr++;
			for (; supervoxel_itr != tmp_supervoxels.end(); supervoxel_itr++) {
				uint32_t node_a, node_b;
				node_a = tmp_supervoxels.begin()->first;
				node_b = supervoxel_itr->first;
				std::pair<uint32_t, uint32_t> tmp = make_pair(node_a, node_b);
				//cout << "\nMerge:";
				//cout << "\n<" << node_a << "> con <" << node_b << ">";
				segmentation.mergeSupervoxel(tmp);
			}
		}
	}

	// BEGIN updating view, recalculate label colors
	obSetItr = objects_set.begin();
	for (; obSetItr != objects_set.end(); obSetItr++) {

/*
		cout << "Oggetto n. " << obSetItr->first;
		obSetItr->second->print();
		cout << "\n";
*/

		map<uint32_t, Supervoxel<PointT>::Ptr> tmp_set =
				obSetItr->second->get_supervoxel_set();
		map<uint32_t, Supervoxel<PointT>::Ptr>::iterator set_itr =
				tmp_set.begin();

		for (; set_itr != tmp_set.end(); set_itr++) {
			convert.str("");
			convert.clear();
			convert << "[" << set_itr->first << "]";
			//viewer->removeText3D(convert.str());
			c = obSetItr->second->get_color();
			//viewer->addText3D(convert.str(), set_itr->second->centroid_, 0.01,
			//		c.r, c.g, c.b);
		}
	}
	// END updating view, recalculate label colors

	// END ALEX
}


// END ALEX

bool Clustering::is_convex(Normal norm1, PointT centroid1, Normal norm2,
		PointT centroid2) const {
	Eigen::Vector3f N1 = norm1.getNormalVector3fMap();
	Eigen::Vector3f C1 = centroid1.getVector3fMap();
	Eigen::Vector3f N2 = norm2.getNormalVector3fMap();
	Eigen::Vector3f C2 = centroid2.getVector3fMap();

	Eigen::Vector3f C = C1 - C2;
	C /= C.norm();

	float cos1 = N1.dot(C);
	float cos2 = N2.dot(C);

	return cos1 >= cos2;
}

float Clustering::normals_diff(Normal norm1, PointT centroid1, Normal norm2,
		PointT centroid2) const {
	Eigen::Vector3f N1 = norm1.getNormalVector3fMap();
	Eigen::Vector3f C1 = centroid1.getVector3fMap();
	Eigen::Vector3f N2 = norm2.getNormalVector3fMap();
	Eigen::Vector3f C2 = centroid2.getVector3fMap();

	Eigen::Vector3f C = C1 - C2;
	C /= C.norm();

	float N1xN2 = N1.cross(N2).norm();
	float N1_C = std::abs(N1.dot(C));
	float N2_C = std::abs(N2.dot(C));

	float delta_g = (N1xN2 + N1_C + N2_C) / 3;

	return delta_g;
}

std::pair<float, float> Clustering::delta_c_g(SupervoxelT::Ptr supvox1,
		SupervoxelT::Ptr supvox2) const {
	float delta_c = 0;
	float * rgb1 = ColorUtilities::mean_color(supvox1);
	float * rgb2 = ColorUtilities::mean_color(supvox2);
	switch (delta_c_type) {
	case LAB_CIEDE00:
		float *lab1, *lab2;
		lab1 = ColorUtilities::rgb2lab(rgb1);
		lab2 = ColorUtilities::rgb2lab(rgb2);
		delta_c = ColorUtilities::lab_ciede00(lab1, lab2);
		delta_c /= LAB_RANGE;
		break;
	case RGB_EUCL:
		delta_c = ColorUtilities::rgb_eucl(rgb1, rgb2);
		delta_c /= RGB_RANGE;
	}

	float delta_g = 0;
	Normal n1 = supvox1->normal_;
	Normal n2 = supvox2->normal_;
	PointT c1 = supvox1->centroid_;
	PointT c2 = supvox2->centroid_;
	switch (delta_g_type) {
	case NORMALS_DIFF:
		delta_g = normals_diff(n1, c1, n2, c2);
		break;
	case CONVEX_NORMALS_DIFF:
		delta_g = normals_diff(n1, c1, n2, c2);
		if (is_convex(n1, c1, n2, c2))
			delta_g *= 0.5;
	}

	std::pair<float, float> ret(delta_c, delta_g);
	return ret;
}

float Clustering::delta(SupervoxelT::Ptr supvox1,
		SupervoxelT::Ptr supvox2) const {

	std::pair<float, float> deltas = delta_c_g(supvox1, supvox2);

	float delta = t_c(deltas.first) + t_g(deltas.second);

//	printf("delta_c = %f | delta_g = %f | delta = %f\n", deltas.first,
//			deltas.second, delta);
	return delta;
}

AdjacencyMapT Clustering::weight2adj(WeightMapT w_map) const {
	AdjacencyMapT adj_map;

	WeightMapT::iterator it = w_map.begin();
	WeightMapT::iterator it_end = w_map.end();
	for (; it != it_end; ++it) {
		adj_map.insert(it->second);
	}

	return adj_map;
}

WeightMapT Clustering::adj2weight(ClusteringT segm,
		AdjacencyMapT adj_map) const {
	WeightMapT w_map;

	AdjacencyMapT::iterator it = adj_map.begin();
	AdjacencyMapT::iterator it_end = adj_map.end();
	for (; it != it_end; ++it) {
		WeightedPairT elem;
		elem.first = -1;
		elem.second = *(it);
		w_map.insert(elem);
	}

	return w_map;
}

void Clustering::init_weights() {
	std::map<std::string, std::pair<float, float> > temp_deltas;
	DeltasDistribT deltas_c;
	DeltasDistribT deltas_g;
	WeightMapT w_new;

	WeightMapT::iterator it = initial_state.weight_map.begin();
	WeightMapT::iterator it_end = initial_state.weight_map.end();
	for (; it != it_end; ++it) {
		uint32_t sup1_id = it->second.first;
		uint32_t sup2_id = it->second.second;
		std::stringstream ids;
		ids << sup1_id << "-" << sup2_id;
		SupervoxelT::Ptr sup1 = initial_state.segments.at(sup1_id);
		SupervoxelT::Ptr sup2 = initial_state.segments.at(sup2_id);
		std::pair<float, float> deltas = delta_c_g(sup1, sup2);
		temp_deltas.insert(
				std::pair<std::string, std::pair<float, float> >(ids.str(),
						deltas));
		deltas_c.insert(deltas.first);
		deltas_g.insert(deltas.second);
	}

	init_merging_parameters(deltas_c, deltas_g);

	it = initial_state.weight_map.begin();
	for (; it != it_end; ++it) {
		uint32_t sup1_id = it->second.first;
		uint32_t sup2_id = it->second.second;
		std::stringstream ids;
		ids << sup1_id << "-" << sup2_id;
		std::pair<float, float> deltas = temp_deltas.at(ids.str());
		float delta = t_c(deltas.first) + t_g(deltas.second);
		w_new.insert(WeightedPairT(delta, it->second));
	}

	initial_state.set_weight_map(w_new);

	init_initial_weights = true;
}

void Clustering::init_merging_parameters(DeltasDistribT deltas_c,
		DeltasDistribT deltas_g) {
	switch (merging_type) {
	case MANUAL_LAMBDA: {
		break;
	}
	case ADAPTIVE_LAMBDA: {
		float mean_c = deltas_mean(deltas_c);
		float mean_g = deltas_mean(deltas_g);
		lambda = mean_g / (mean_c + mean_g);
		break;
	}
	case EQUALIZATION: {
		cdf_c = compute_cdf(deltas_c);
		cdf_g = compute_cdf(deltas_g);
	}
	}
}

std::map<short, float> Clustering::compute_cdf(DeltasDistribT dist) {
	std::map<short, float> cdf;
	int bins[bins_num];// = { };

	DeltasDistribT::iterator d_itr, d_itr_end;
	d_itr = dist.begin();
	d_itr_end = dist.end();
	int n = dist.size();
	for (; d_itr != d_itr_end; ++d_itr) {
		float d = *d_itr;
		short bin = std::floor(d * bins_num);
		if (bin == bins_num)
			bin--;
		bins[bin]++;
	}

	for (short i = 0; i < bins_num; i++) {
		float v = 0;
		for (short j = 0; j <= i; j++)
			v += bins[j];
		v /= n;
		cdf.insert(std::pair<short, float>(i, v));
	}

	return cdf;
}

float Clustering::t_c(float delta_c) const {
	float ret = 0;
	switch (merging_type) {
	case MANUAL_LAMBDA: {
		ret = lambda * delta_c;
		break;
	}
	case ADAPTIVE_LAMBDA: {
		ret = lambda * delta_c;
		break;
	}
	case EQUALIZATION: {
		short bin = std::floor(delta_c * bins_num);
		if (bin == bins_num)
			bin--;
		ret = cdf_c.at(bin) / 2;
	}
	}
	return ret;
}

float Clustering::t_g(float delta_g) const {
	float ret = 0;
	switch (merging_type) {
	case MANUAL_LAMBDA: {
		ret = (1 - lambda) * delta_g;
		break;
	}
	case ADAPTIVE_LAMBDA: {
		ret = (1 - lambda) * delta_g;
		break;
	}
	case EQUALIZATION: {
		short bin = std::floor(delta_g * bins_num);
		ret = cdf_g.at(bin) / 2;
	}
	}
	return ret;
}

void Clustering::cluster(ClusteringState start, float threshold) {
	state = start;

	WeightedPairT next;
	while (!state.weight_map.empty()
			&& (next = state.get_first_weight(), next.first < threshold)) {
		console::print_debug("left: %de/%dp - w: %f - [%d, %d]...",
				state.weight_map.size(), state.segments.size(), next.first,
				next.second.first, next.second.second);
		merge(next.second);
		console::print_debug("OK\n");
	}
}

void Clustering::merge(std::pair<uint32_t, uint32_t> supvox_ids) {
	SupervoxelT::Ptr sup1 = state.segments.at(supvox_ids.first);
	SupervoxelT::Ptr sup2 = state.segments.at(supvox_ids.second);
	SupervoxelT::Ptr sup_new = boost::make_shared<SupervoxelT>();

	*(sup_new->voxels_) = *(sup1->voxels_) + *(sup2->voxels_);
	*(sup_new->normals_) = *(sup1->normals_) + *(sup2->normals_);

	PointT new_centr;
	computeCentroid(*(sup_new->voxels_), new_centr);
	sup_new->centroid_ = new_centr;

	Eigen::Vector4f new_norm;
	float new_curv;
	computePointNormal(*(sup_new->voxels_), new_norm, new_curv);
	flipNormalTowardsViewpoint(sup_new->centroid_, 0, 0, 0, new_norm);
	new_norm[3] = 0.0f;
	new_norm.normalize();
	sup_new->normal_.normal_x = new_norm[0];
	sup_new->normal_.normal_y = new_norm[1];
	sup_new->normal_.normal_z = new_norm[2];
	sup_new->normal_.curvature = new_curv;

	state.segments.erase(supvox_ids.first);
	state.segments.erase(supvox_ids.second);
	state.segments.insert(
			std::pair<uint32_t, SupervoxelT::Ptr>(supvox_ids.first, sup_new));

	WeightMapT new_map;

	WeightMapT::iterator it = state.weight_map.begin();
	++it;
	WeightMapT::iterator it_end = state.weight_map.end();
	for (; it != it_end; ++it) {
		std::pair<uint32_t, uint32_t> curr_ids = it->second;
		if (curr_ids.first == supvox_ids.first
				|| curr_ids.second == supvox_ids.first) {
			if (!contains(new_map, curr_ids.first, curr_ids.second)) {
				float w = delta(state.segments.at(curr_ids.first),
						state.segments.at(curr_ids.second));
				new_map.insert(WeightedPairT(w, curr_ids));
			}
		} else if (curr_ids.first == supvox_ids.second) {
			curr_ids.first = supvox_ids.first;
			if (!contains(new_map, curr_ids.first, curr_ids.second)) {
				float w = delta(state.segments.at(curr_ids.first),
						state.segments.at(curr_ids.second));
				new_map.insert(WeightedPairT(w, curr_ids));
			}
		} else if (curr_ids.second == supvox_ids.second) {
			if (curr_ids.first < supvox_ids.first)
				curr_ids.second = supvox_ids.first;
			else {
				curr_ids.second = curr_ids.first;
				curr_ids.first = supvox_ids.first;
			}
			if (!contains(new_map, curr_ids.first, curr_ids.second)) {
				float w = delta(state.segments.at(curr_ids.first),
						state.segments.at(curr_ids.second));
				new_map.insert(WeightedPairT(w, curr_ids));
			}
		} else {
			new_map.insert(*it);
		}
	}
	state.weight_map = new_map;
}

ClusteringState Clustering::getState(){
	return state;
}

void Clustering::clear_adjacency(AdjacencyMapT * adjacency) {
	AdjacencyMapT::iterator it = adjacency->begin();
	AdjacencyMapT::iterator it_end = adjacency->end();
	while (it != it_end) {
		if (it->first > it->second) {
			adjacency->erase(it++);
		} else {
			++it;
		}
	}
//	for (; it != it_end; ++it) {
//		uint32_t key = it->first;
//		uint32_t value = it->second;
//		std::pair<AdjacencyMapT::iterator, AdjacencyMapT::iterator> range =
//				adjacency->equal_range(value);
//		AdjacencyMapT::iterator val_it = range.first;
//		for (; val_it != range.second; ++val_it) {
//			if (val_it->second == key) {
//				adjacency->erase(val_it);
//				break;
//			}
//		}
//	}
}

bool Clustering::contains(WeightMapT w, uint32_t i1, uint32_t i2) {
	WeightMapT::iterator it = w.begin();
	WeightMapT::iterator it_end = w.end();
	for (; it != it_end; ++it) {
		std::pair<uint32_t, uint32_t> ids = it->second;
		if (ids.first == i1 && ids.second == i2)
			return true;
	}
	return false;
}

float Clustering::deltas_mean(DeltasDistribT deltas) {
	DeltasDistribT::iterator d_itr, d_itr_end;
	d_itr = deltas.begin();
	d_itr_end = deltas.end();
	float count = 0;
	float mean_d = 0;
	for (; d_itr != d_itr_end; ++d_itr) {
		float delta = *d_itr;
		count++;

		mean_d = mean_d + (1 / count) * (delta - mean_d);
	}
	return mean_d;
}

Clustering::Clustering() {
	set_delta_c(LAB_CIEDE00);
	set_delta_g(NORMALS_DIFF);
	set_merging(ADAPTIVE_LAMBDA);
	set_initial_state = false;
	init_initial_weights = false;
}

Clustering::Clustering(ColorDistance c, GeometricDistance g,
		MergingCriterion m) {
	set_delta_c(c);
	set_delta_g(g);
	set_merging(m);
	set_initial_state = false;
	init_initial_weights = false;
}

void Clustering::set_merging(MergingCriterion m) {
	merging_type = m;
	lambda = 0.5;
	bins_num = 500;
	init_initial_weights = false;
}

void Clustering::set_lambda(float l) {
	if (merging_type != MANUAL_LAMBDA)
		throw std::logic_error(
				"Lambda can be set only if the merging criterion is set to MANUAL_LAMBDA");
	if (l < 0 || l > 1)
		throw std::invalid_argument("Argument outside range [0, 1]");
	lambda = l;
	init_initial_weights = false;
}

void Clustering::set_bins_num(short b) {
	if (merging_type != EQUALIZATION)
		throw std::logic_error(
				"Bins number can be set only if the merging criterion is set to EQUALIZATION");
	if (b < 0)
		throw std::invalid_argument("Argument lower than 0");
	bins_num = b;
	init_initial_weights = false;
}

void Clustering::set_initialstate(ClusteringT segm, AdjacencyMapT adj) {
	clear_adjacency(&adj);
	ClusteringState init_state(segm, adj2weight(segm, adj));
	initial_state = init_state;
	state = init_state;
	set_initial_state = true;
	init_initial_weights = false;
}

std::pair<ClusteringT, AdjacencyMapT> Clustering::get_currentstate() const {
	std::pair<ClusteringT, AdjacencyMapT> ret;
	ret.first = state.segments;
	ret.second = weight2adj(state.weight_map);
	return ret;
}

PointCloud<PointT>::Ptr Clustering::get_colored_cloud() const {
	return label2color(get_labeled_cloud());
}

PointCloud<PointXYZL>::Ptr Clustering::get_labeled_cloud() const {
	PointCloud<PointXYZL>::Ptr label_cloud(new PointCloud<PointXYZL>);

	ClusteringT::const_iterator it = state.segments.begin();
	ClusteringT::const_iterator it_end = state.segments.end();

	uint32_t current_l = 0;
	for (; it != it_end; ++it) {
		PointCloud<PointT> cloud = *(it->second->voxels_);
		PointCloud<PointT>::iterator it_cloud = cloud.begin();
		PointCloud<PointT>::iterator it_cloud_end = cloud.end();
		for (; it_cloud != it_cloud_end; ++it_cloud) {
			PointXYZL p;
			p.x = it_cloud->x;
			p.y = it_cloud->y;
			p.z = it_cloud->z;
			p.label = current_l;
			label_cloud->push_back(p);
		}
		current_l++;
	}

	return label_cloud;
}

void Clustering::cluster(float threshold) {
	if (!set_initial_state)
		throw std::logic_error("Cannot call 'cluster' before "
				"setting an initial state with 'set_initialstate'");

	if (!init_initial_weights)
		init_weights();

	cluster(initial_state, threshold);
}
std::map<float, performanceSet> Clustering::all_thresh(
		PointCloud<PointLT>::Ptr ground_truth, float start_thresh,
		float end_thresh, float step_thresh) {
	if (start_thresh < 0 || start_thresh > 1 || end_thresh < 0 || end_thresh > 1
			|| step_thresh < 0 || step_thresh > 1) {
		throw std::out_of_range(
				"start_thresh, end_thresh and/or step_thresh outside of range [0, 1]");
	}
	if (start_thresh > end_thresh) {
		console::print_warn(
				"Start threshold greater then end threshold, inverting.\n");
		float temp = end_thresh;
		end_thresh = start_thresh;
		start_thresh = temp;
	}

	console::print_info("Testing thresholds from %f to %f (step %f)\n",
			start_thresh, end_thresh, step_thresh);

	std::map<float, performanceSet> thresholds;
	cluster(start_thresh);
	Testing test(get_labeled_cloud(), ground_truth);
	performanceSet p = test.eval_performance();
	thresholds.insert(std::pair<float, performanceSet>(start_thresh, p));
	console::print_info("<T, Fscore, voi, wov> = <%f, %f, %f, %f>\n",
			start_thresh, p.fscore, p.voi, p.wov);

	for (float t = start_thresh + step_thresh; t <= end_thresh; t +=
			step_thresh) {
		cluster(state, t);
		test.set_segm(get_labeled_cloud());
		p = test.eval_performance();
		thresholds.insert(std::pair<float, performanceSet>(t, p));
		console::print_info("<T, Fscore, voi, wov> = <%f, %f, %f, %f>\n", t,
				p.fscore, p.voi, p.wov);
	}

	return thresholds;
}

std::map<float, performanceSet> Clustering::all_thresh_v2(
		ClusteringT supervoxel_clusters, AdjacencyMapT label_adjacency,
		PointCloud<PointLT>::Ptr ground_truth, float start_thresh,
		float end_thresh, float step_thresh, float toll_multiplier, bool CVX, bool GA) {
	if (start_thresh < 0 || start_thresh > 1 || end_thresh < 0 || end_thresh > 1
			|| step_thresh < 0 || step_thresh > 1) {
		throw std::out_of_range(
				"start_thresh, end_thresh and/or step_thresh outside of range [0, 1]");
	}
	if (start_thresh > end_thresh) {
		console::print_warn(
				"Start threshold greater then end threshold, inverting.\n");
		float temp = end_thresh;
		end_thresh = start_thresh;
		start_thresh = temp;
	}

	console::print_info("Testing thresholds from %f to %f (step %f)\n",
			start_thresh, end_thresh, step_thresh);

	std::map<float, performanceSet> thresholds;
/*	Clustering segmentation;

	if(CVX)
		segmentation.set_delta_g(CONVEX_NORMALS_DIFF);

	segmentation.set_initialstate(supervoxel_clusters, label_adjacency);

	segmentation.cluster(start_thresh);

	AdjacencyMapT tmp_adjacency = segmentation.get_currentstate().second;
	if(GA)
		analyze_graph(segmentation, tmp_adjacency, toll_multiplier);

	Testing test(segmentation.get_labeled_cloud(), ground_truth);

	performanceSet p = test.eval_performance();
	thresholds.insert(std::pair<float, performanceSet>(start_thresh, p));
	console::print_info("<T, Fscore, voi, wov> = <%f, %f, %f, %f>\n",
			start_thresh, p.fscore, p.voi, p.wov);
*/
	performanceSet p;
	for (float t = start_thresh/* + step_thresh*/; t <= end_thresh; t +=
			step_thresh) {
		Clustering segmentation;
		if(CVX)
			segmentation.set_delta_g(CONVEX_NORMALS_DIFF);
		segmentation.set_initialstate(supervoxel_clusters, label_adjacency);

		segmentation.cluster(t);
		AdjacencyMapT tmp_adjacency = segmentation.get_currentstate().second;

		if(GA)
			if(tmp_adjacency.size() > 2){
				analyze_graph(segmentation, tmp_adjacency, toll_multiplier);
				//t_segm(segmentation.get_labeled_cloud());
				Testing test(segmentation.get_labeled_cloud(), ground_truth);
				p = test.eval_performance();
			}
			else {
				performanceSet tmp;
				p = tmp;
			}
		else{
			Testing test(segmentation.get_labeled_cloud(), ground_truth);
			p = test.eval_performance();
		}


		thresholds.insert(std::pair<float, performanceSet>(t, p));
		console::print_info("<T, Fscore, voi, wov> = <%f, %f, %f, %f>\n", t,
				p.fscore, p.voi, p.wov);
	}

	return thresholds;
}

std::pair<float, performanceSet> Clustering::best_thresh(
		PointCloud<PointLT>::Ptr ground_truth, float start_thresh,
		float end_thresh, float step_thresh) {
	std::map<float, performanceSet> thresholds = all_thresh(ground_truth,
			start_thresh, end_thresh, step_thresh);
	return best_thresh(thresholds);
}

std::pair<float, performanceSet> Clustering::best_thresh(
		std::map<float, performanceSet> all_thresh) {
	float best_t = 0;
	performanceSet best_performance;

	std::map<float, performanceSet>::iterator it = all_thresh.begin();

	for (; it != all_thresh.end(); ++it) {
		if (it->second.fscore > best_performance.fscore) {
			best_performance = it->second;
			best_t = it->first;
		}
	}

	return std::pair<float, performanceSet>(best_t, best_performance);
}

void Clustering::test_all() const {
	ColorUtilities::rgb_test();
	ColorUtilities::lab_test();
	ColorUtilities::convert_test();

//	DeltasDistribT d;
//	for(int i = 0; i<10; i++)
//		d.insert(i);
//	printf("*** Mean deltas test ***\nMean = %f\n", deltas_mean(d));
}

PointCloud<PointT>::Ptr Clustering::label2color(
		PointCloud<PointLT>::Ptr label_cloud) {
	PointCloud<PointT>::Ptr colored_cloud(new PointCloud<PointT>);
	PointCloud<PointLCT>::Ptr temp_cloud(new PointCloud<PointLCT>);

	copyPointCloud(*label_cloud, *temp_cloud);

	PointCloud<PointLCT>::iterator it = temp_cloud->begin();
	PointCloud<PointLCT>::iterator it_end = temp_cloud->end();

	for (; it != it_end; ++it) {
		uint8_t * rgb = ColorUtilities::get_glasbey(it->label);
		it->r = rgb[0];
		it->g = rgb[1];
		it->b = rgb[2];
	}

	copyPointCloud(*temp_cloud, *colored_cloud);
	return colored_cloud;
}

PointCloud<PointLT>::Ptr Clustering::color2label(
		PointCloud<PointT>::Ptr colored_cloud) {
	PointCloud<PointLT>::Ptr label_cloud(new PointCloud<PointLT>);
	PointCloud<PointLCT>::Ptr temp_cloud(new PointCloud<PointLCT>);
	std::map<float, uint32_t> mappings;
	copyPointCloud(*colored_cloud, *temp_cloud);

	PointCloud<PointLCT>::iterator it = temp_cloud->begin();
	PointCloud<PointLCT>::iterator it_end = temp_cloud->end();

	uint32_t i = 0;
	for (; it != it_end; ++it) {
		if (mappings.count(it->rgb) != 0)
			it->label = mappings.at(it->rgb);
		else {
			it->label = i;
			mappings.insert(std::pair<float, uint32_t>(it->rgb, i));
			i++;
		}
	}

	copyPointCloud(*temp_cloud, *label_cloud);
	return label_cloud;
}
