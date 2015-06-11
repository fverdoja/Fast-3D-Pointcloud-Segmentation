/*
 * Clustering.h
 *
 *  Created on: 19/05/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 */

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include "ColorUtilities.h"
#include "ClusteringState.h"

using namespace pcl;

typedef PointXYZRGBA PointT;
typedef Supervoxel<PointT> SupervoxelT;
typedef std::map<uint32_t, SupervoxelT::Ptr> ClusteringT;
typedef std::multimap<uint32_t, uint32_t> AdjacencyMapT;

enum ColorDistance {
	LAB_CIEDE00, RGB_EUCL
};
enum GeometricDistance {
	NORMALS_DIFF
};

class Clustering {
	ColorDistance delta_c_type;
	GeometricDistance delta_g_type;
	float lambda;
	bool set_initial_state;
	ClusteringState initial_state;
	ClusteringState state;

	float normals_diff(Normal norm1, PointT centroid1, Normal norm2,
			PointT centroid2) const;
	float delta(SupervoxelT::Ptr supvox1, SupervoxelT::Ptr supvox2) const;
	AdjacencyMapT weight2adj(WeightMapT w_map) const;
	WeightMapT adj2weight(ClusteringT segm, AdjacencyMapT adj_map) const;
	void merge(std::pair<uint32_t, uint32_t> supvox_ids);
	static void clear_adjacency(AdjacencyMapT * adjacency);
	static bool contains(WeightMapT w, uint32_t i1, uint32_t i2);

public:
	Clustering();
	Clustering(ColorDistance c, GeometricDistance g, float l);

	void set_delta_c(ColorDistance d) {
		delta_c_type = d;
	}
	void set_delta_g(GeometricDistance d) {
		delta_g_type = d;
	}
	void set_lambda(float l);
	void set_initialstate(ClusteringT segm, AdjacencyMapT adj);

	ColorDistance get_delta_c() const {
		return delta_c_type;
	}
	GeometricDistance get_delta_g() const {
		return delta_g_type;
	}
	float get_lambda() const {
		return lambda;
	}
	std::pair<ClusteringT, AdjacencyMapT> get_currentstate() const;

	PointCloud<PointT>::Ptr get_colored_cloud() const;

	void cluster(float threshold);

	void test_all() const;
};

#endif /* CLUSTERING_H_ */
