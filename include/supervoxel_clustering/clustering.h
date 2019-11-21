/*
 * clustering.h
 *
 *  Created on: 19/05/2015
 *      Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 *
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2015, Francesco Verdoja
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its 
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include "color_utilities.h"
#include "clustering_state.h"
#include "testing.h"

typedef pcl::Normal Normal;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointXYZRGBL PointLCT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::Supervoxel<PointT> SupervoxelT;
typedef std::map<uint32_t, SupervoxelT::Ptr> ClusteringT;
typedef std::multimap<uint32_t, uint32_t> AdjacencyMapT;
typedef std::multiset<float> DeltasDistribT;

enum ColorDistance {
    LAB_CIEDE00, RGB_EUCL
};

enum GeometricDistance {
    NORMALS_DIFF, CONVEX_NORMALS_DIFF
};

enum MergingCriterion {
    MANUAL_LAMBDA, ADAPTIVE_LAMBDA, EQUALIZATION
};

/**
 * This class performs the hierarchical supervoxel clustering as described in 
 * [1]. For an example of use please refer to the file 
 * supervoxel_segmentation.cpp
 * 
 * [1] F. Verdoja, D. Thomas, and A. Sugimoto, "Fast 3D point cloud segmentation 
 *     using supervoxels with geometry and color for 3D scene understanding," in 
 *     IEEE International Conference on Multimedia and Expo (ICME 2017), Hong 
 *     Kong, 2017, pp. 1285–1290.
 */
class Clustering {
    ColorDistance delta_c_type;
    GeometricDistance delta_g_type;
    MergingCriterion merging_type;
    float lambda;
    short bins_num;
    std::map<short, float> cdf_c, cdf_g;
    bool set_initial_state, init_initial_weights;
    ClusteringState initial_state, state;

    bool is_convex(Normal norm1, PointT centroid1, Normal norm2,
            PointT centroid2) const;
    float normals_diff(Normal norm1, PointT centroid1, Normal norm2,
            PointT centroid2) const;
    std::pair<float, float> delta_c_g(SupervoxelT::Ptr supvox1,
            SupervoxelT::Ptr supvox2) const;
    float delta(SupervoxelT::Ptr supvox1, SupervoxelT::Ptr supvox2) const;
    AdjacencyMapT weight2adj(WeightMapT w_map) const;
    WeightMapT adj2weight(ClusteringT segm, AdjacencyMapT adj_map) const;
    void init_weights();
    void init_merging_parameters(DeltasDistribT deltas_c,
            DeltasDistribT deltas_g);
    std::map<short, float> compute_cdf(DeltasDistribT dist);
    float t_c(float delta_c) const;
    float t_g(float delta_g) const;
    void cluster(ClusteringState start, float threshold);
    void merge(std::pair<uint32_t, uint32_t> supvox_ids);

    static void clear_adjacency(AdjacencyMapT * adjacency);
    static bool contains(WeightMapT w, uint32_t i1, uint32_t i2);
    static float deltas_mean(DeltasDistribT deltas);

public:

    Clustering();
    Clustering(ColorDistance c, GeometricDistance g, MergingCriterion m);

    /**
     * Set the type of color distance to be used
     * 
     * @param d a type of color distance
     */
    void set_delta_c(ColorDistance d) {
        delta_c_type = d;
    }

    /**
     * Set the type of geometric distance to be used
     * 
     * @param d a type of geometric distance
     */
    void set_delta_g(GeometricDistance d) {
        delta_g_type = d;
    }

    void set_merging(MergingCriterion m);
    void set_lambda(float l);
    void set_bins_num(short b);
    void set_initialstate(ClusteringT segm, AdjacencyMapT adj);

    /**
     * Get the type of color distance used
     * 
     * @return a type of color distance
     */
    ColorDistance get_delta_c() const {
        return delta_c_type;
    }

    /**
     * Get the type of geometric distance used
     * 
     * @return a type of geometric distance
     */
    GeometricDistance get_delta_g() const {
        return delta_g_type;
    }

    /**
     * Get the type of merging approach used for unification of color and 
     * geometric distances
     * 
     * @return a type of merging approach
     */
    MergingCriterion get_merging() const {
        return merging_type;
    }

    /**
     * Get the current value of lambda
     * 
     * @return the value of lambda
     */
    float get_lambda() const {
        return lambda;
    }

    /**
     * Get the number of bins used for histogram equalization 
     * 
     * @return the number of bins
     */
    short get_bins_num() const {
        return bins_num;
    }

    std::pair<ClusteringT, AdjacencyMapT> get_currentstate() const;

    PointCloudT::Ptr get_colored_cloud() const;
    PointLCloudT::Ptr get_labeled_cloud() const;

    void cluster(float threshold);

    std::map<float, performanceSet> all_thresh(
            PointLCloudT::Ptr ground_truth, float start_thresh,
            float end_thresh, float step_thresh);
    std::pair<float, performanceSet> best_thresh(
            PointLCloudT::Ptr ground_truth, float start_thresh,
            float end_thresh, float step_thresh);
    std::pair<float, performanceSet> best_thresh(
            std::map<float, performanceSet> all_thresh);

    void test_all() const;

    static PointCloudT::Ptr label2color(
            PointLCloudT::Ptr label_cloud);
    static PointLCloudT::Ptr color2label(
            PointCloudT::Ptr colored_cloud);
};

#endif /* CLUSTERING_H_ */
