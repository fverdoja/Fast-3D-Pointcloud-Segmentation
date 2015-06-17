/*
 * ClusteringState.h
 *
 *  Created on: 01/giu/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 */

#ifndef CLUSTERINGSTATE_H_
#define CLUSTERINGSTATE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

using namespace pcl;

typedef PointXYZRGBA PointT;
typedef Supervoxel<PointT> SupervoxelT;
typedef std::map<uint32_t, SupervoxelT::Ptr> ClusteringT;
typedef std::multimap<float, std::pair<uint32_t, uint32_t> > WeightMapT;
typedef std::pair<float, std::pair<uint32_t, uint32_t> > WeightedPairT;

class ClusteringState {
	friend class Clustering;

	ClusteringT segments;
	WeightMapT weight_map;

public:
	ClusteringState() {
	}

	ClusteringState(ClusteringT s, WeightMapT w);

	ClusteringT get_segments() const {
		return segments;
	}
	void set_segments(ClusteringT s) {
		segments = s;
	}
	WeightMapT get_weight_map() const {
		return weight_map;
	}
	void set_weight_map(WeightMapT w) {
		weight_map = w;
	}
	WeightedPairT get_first_weight() const {
		return *(weight_map.begin());
	}
};

#endif /* CLUSTERINGSTATE_H_ */
