/*
 * Clustering.cpp
 *
 *  Created on: 19/05/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 */

#include "Clustering.h"

/*
 * IMPLEMENTATIONS OF CONSTRUCTORS OF CLASS Clustering
 */

Clustering::Clustering() {
	set_delta_c(LAB_CIEDE00);
	set_delta_g(NORMALS_DIFF);
	set_lambda(0.5);
	set_initial_state = false;
}

Clustering::Clustering(ColorDistance c, GeometricDistance g, float l) {
	set_delta_c(c);
	set_delta_g(g);
	set_lambda(l);
	set_initial_state = false;
}

/*
 * IMPLEMENTATIONS OF PUBLIC METHODS OF CLASS Clustering
 */

void Clustering::set_lambda(float l) {
	if (l < 0 || l > 1)
		throw std::invalid_argument("Argument outside range [0, 1]");
	lambda = l;
}

void Clustering::set_initialstate(ClusteringT segm, AdjacencyMapT adj) {
	clear_adjacency(&adj);
	ClusteringState init_state(segm, adj2weight(segm, adj));
	initial_state = init_state;
	state = init_state;
	set_initial_state = true;
}

void Clustering::cluster(float threshold) {
	if (!set_initial_state)
		throw std::logic_error("Cannot call 'cluster' before "
				"setting an initial state with 'set_initialstate'");

	state = initial_state;

	WeightedPairT next;
	while (!state.weight_map.empty()
			&& (next = state.get_first_weight(), next.first < threshold)) {
		printf("left: %de/%dp - w: %f - [%d, %d]\t", state.weight_map.size(),
				state.segments.size(), next.first, next.second.first,
				next.second.second);
		merge(next.second);
		printf("OK\n");
	}
}

PointCloud<PointT>::Ptr Clustering::get_colored_cloud() const {
	PointCloud<PointT>::Ptr colored_cloud(new PointCloud<PointT>);

	ClusteringT::const_iterator it = state.segments.begin();
	ClusteringT::const_iterator it_end = state.segments.end();

	for (; it != it_end; ++it) {
		uint8_t r, g, b;
		r = rand() % 256;
		g = rand() % 256;
		b = rand() % 256;
		PointCloud<PointT> cloud = *(it->second->voxels_);
		PointCloud<PointT>::iterator it_cloud = cloud.begin();
		PointCloud<PointT>::iterator it_cloud_end = cloud.end();
		for (; it_cloud != it_cloud_end; ++it_cloud) {
			it_cloud->r = r;
			it_cloud->g = g;
			it_cloud->b = b;
		}
		*(colored_cloud) += cloud;
	}

	return colored_cloud;
}

std::pair<ClusteringT, AdjacencyMapT> Clustering::get_currentstate() const {
	std::pair<ClusteringT, AdjacencyMapT> ret;
	ret.first = state.segments;
	ret.second = weight2adj(state.weight_map);
	return ret;
}

void Clustering::test_all() const {
	ColorUtilities::rgb_test();
	ColorUtilities::lab_test();
	ColorUtilities::convert_test();
}

/*
 * IMPLEMENTATIONS OF PRIVATE METHODS OF CLASS Clustering
 */

float Clustering::delta(SupervoxelT::Ptr supvox1,
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
	switch (delta_g_type) {
	case NORMALS_DIFF:
		Normal n1 = supvox1->normal_;
		Normal n2 = supvox2->normal_;
		PointT c1 = supvox1->centroid_;
		PointT c2 = supvox2->centroid_;
		delta_g = normals_diff(n1, c1, n2, c2);
	}

//	printf("delta_c = %f | delta_g = %f\n", delta_c, delta_g);

	float delta = lambda * delta_c + (1 - lambda) * delta_g;
	return delta;
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
		elem.first = delta(segm.at(it->first), segm.at(it->second));
		elem.second = *(it);
		w_map.insert(elem);
	}

	return w_map;
}

void Clustering::merge(std::pair<uint32_t, uint32_t> supvox_ids) {
	SupervoxelT::Ptr sup1 = state.segments.at(supvox_ids.first);
	SupervoxelT::Ptr sup2 = state.segments.at(supvox_ids.second);

	*(sup1->voxels_) += *(sup2->voxels_);
	*(sup1->normals_) += *(sup2->normals_);

	PointT new_centr;
	computeCentroid(*(sup1->voxels_), new_centr);
	sup1->centroid_ = new_centr;

	Eigen::Vector4f new_norm;
	float new_curv;
	computePointNormal(*(sup1->voxels_), new_norm, new_curv);
	flipNormalTowardsViewpoint(sup1->centroid_, 0, 0, 0, new_norm);
	new_norm[3] = 0.0f;
	new_norm.normalize();
	sup1->normal_.normal_x = new_norm[0];
	sup1->normal_.normal_y = new_norm[1];
	sup1->normal_.normal_z = new_norm[2];
	sup1->normal_.curvature = new_curv;

	state.segments.erase(supvox_ids.second);

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

float Clustering::normals_diff(Normal norm1, PointT centroid1, Normal norm2,
		PointT centroid2) const {
	Eigen::Vector3f N1 = norm1.getNormalVector3fMap();
	Eigen::Vector3f C1 = centroid1.getVector3fMap();
	Eigen::Vector3f N2 = norm2.getNormalVector3fMap();
	Eigen::Vector3f C2 = centroid2.getVector3fMap();

	Eigen::Vector3f C = C2 - C1;
	C /= C.norm();

	float N1xN2 = N1.cross(N2).norm();
	float N1_C = std::abs(N1.dot(C));
	float N2_C = std::abs(N2.dot(C));

	float delta_g = (N1xN2 + N1_C + N2_C) / 3;

//	std::cout << "c1 =\n" << C1 << "\n";
//	std::cout << "c2 =\n" << C2 << "\n";
//	std::cout << "C =\n" << C << "\n";
//	std::cout << "N1_C =\n" << N1_C << "\n";
//	std::cout << "N2_C =\n" << N2_C << "\n";
//	std::cout <<"n1 =\n"<< N1 << "\n";
//	std::cout <<"n2 =\n"<< N2 << "\n";
//	std::cout <<"n1 x n2 =\n"<< N1.cross(N2) << "\n|n1 x n2| = " << N1xN2 << "\n";

	return delta_g;
}
