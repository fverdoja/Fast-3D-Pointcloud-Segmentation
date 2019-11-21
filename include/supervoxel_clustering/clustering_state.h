/*
 * clustering_state.h
 *
 *  Created on: 01/06/2015
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

#ifndef CLUSTERINGSTATE_H_
#define CLUSTERINGSTATE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Supervoxel<PointT> SupervoxelT;
typedef std::map<uint32_t, SupervoxelT::Ptr> ClusteringT;
typedef std::multimap<float, std::pair<uint32_t, uint32_t> > WeightMapT;
typedef std::pair<float, std::pair<uint32_t, uint32_t> > WeightedPairT;

/**
 * Data structure representing a state of the clustering process. It holds 
 * information about the graph nodes and edge weights.
 *  
 * Each node represents a region of the segmentation and has a label assigned.
 * 
 * Edge weights are represented as a map where the cell addressed by two labels 
 * contains the weight of the edge connecting the nodes identified by those 
 * labels. This map is sorted from the smallest weight to the biggest.
 */
class ClusteringState {
    friend class Clustering;

    ClusteringT segments;
    WeightMapT weight_map;

public:

    ClusteringState() {
    }
    
    ClusteringState(ClusteringT s, WeightMapT w);

    /**
     * Get all nodes in the graph
     * 
     * @return a map containing the regions of the segmentations, each 
     *         identified by an unique label
     */
    ClusteringT get_segments() const {
        return segments;
    }

    /**
     * Set values for all nodes in the graph
     * 
     * @param s a map containing the regions of the segmentations, each 
     *          identified by an unique label
     */
    void set_segments(ClusteringT s) {
        segments = s;
    }

    /**
     * Get the weight map of the graph
     * 
     * @return a multimap containing the edge weights of the graph
     */
    WeightMapT get_weight_map() const {
        return weight_map;
    }

    /**
     * Set all edge weights to the given map
     * 
     * @param w a multimap containing the edge weights of the graph
     */
    void set_weight_map(WeightMapT w) {
        weight_map = w;
    }

    /**
     * Get the pair of nodes connected by the edge having the smallest weight
     * 
     * @return a pair of labels corresponding to the two most similar nodes in 
     *         the graph, and the weight of the edge connecting them
     */
    WeightedPairT get_first_weight() const {
        return *(weight_map.begin());
    }
};

#endif /* CLUSTERINGSTATE_H_ */
