/*
 * Testing.h
 *
 *  Created on: 13/07/2015
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

#ifndef TESTING_H_
#define TESTING_H_

#include <map>
#include <set>
#include <algorithm>  // for std::set_intersection, std::set_union, std::sort
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <boost/make_shared.hpp>

using namespace pcl;

typedef PointXYZL PointLT;
typedef PointCloud<PointLT> PointLCloudT;
typedef std::vector<PointLT, Eigen::aligned_allocator<PointLT> > PointLVectorT;
typedef std::map<uint32_t, PointLCloudT::Ptr> labelMapT;

struct compareXYZ {
	bool operator()(PointLT const &p1, PointLT const &p2) const {
		if (p1.x != p2.x)
			return p1.x < p2.x;
		if (p1.y != p2.y)
			return p1.y < p2.y;
		return p1.z < p2.z;
	}
};

struct performanceSet {
	performanceSet() :
			voi(0), precision(0), recall(0), fscore(0), wov(0) {
	}
	float voi, precision, recall, fscore, wov;
};

class Testing {
	PointLCloudT::Ptr segm, truth;
	labelMapT segm_labels, truth_labels;
	Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> inter_matrix;
	Eigen::Array<int64_t, 1, Eigen::Dynamic> matches;
	float precision, recall, fscore, voi, wov;
	bool is_set_segm, is_set_truth;

	void init_performance();
	labelMapT label_map(PointLCloudT::Ptr in);
	void compute_intersections();
	PointLCloudT::Ptr extract_label_cloud(PointLCloudT::Ptr c,
			uint32_t label) const;
	size_t count_intersect(PointLCloudT::Ptr c1, PointLCloudT::Ptr c2) const;
	size_t count_union(PointLCloudT::Ptr c1, PointLCloudT::Ptr c2) const;

	Testing() {
		init_performance();
	}

public:
	Testing(PointLCloudT::Ptr s, PointLCloudT::Ptr t);

	float eval_precision();
	float eval_recall();
	float eval_fscore();
	float eval_voi();
	float eval_wov();
	performanceSet eval_performance();

	PointLCloudT::Ptr get_segm() const {
		return segm;
	}
	PointLCloudT::Ptr get_truth() const {
		return truth;
	}

	void set_segm(PointLCloudT::Ptr s);
	void set_truth(PointLCloudT::Ptr t);
};

#endif /* TESTING_H_ */
