/*
 * supervoxel.h
 *
 *  Created on: 20/04/2020
 *      Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 *
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, Francesco Verdoja
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

#ifndef SUPERVOXEL_H_
#define SUPERVOXEL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <Eigen/Dense>

typedef pcl::PointXYZRGBA PointT;

/**
 * Data structure representing a supervoxel or segmentation region. It extends
 * the standard pcl::Supervoxel by adding friction and variances for all the 
 * region properties (i.e., friction, color, position, and normal)
 */
class Supervoxel : public pcl::Supervoxel<PointT> {
  public: 
    Supervoxel() : 
      pcl::Supervoxel<PointT>(),
      frictions_(new pcl::PointCloud<pcl::PointXYZI> ()),
      mean_(),
      covariance_()
      {}
    Supervoxel(const pcl::Supervoxel<PointT>& s) : 
      pcl::Supervoxel<PointT>(s),
      frictions_(new pcl::PointCloud<pcl::PointXYZI> ()),
      mean_(),
      covariance_()
      {}
    
    typedef boost::shared_ptr<Supervoxel> Ptr;
    typedef boost::shared_ptr<const Supervoxel> ConstPtr;

    /** \brief Computes the mean vector and covariance matrix */
    void compute_statistics();

    /** \brief The friction calculated for the voxels contained in the supervoxel */
    float friction_;
    /** \brief The friction variance calculated for the voxels contained in the supervoxel */
    float friction_variance_;
    /** \brief A Pointcloud of the voxels along the haptic track in the supervoxel */
    pcl::PointCloud<pcl::PointXYZI>::Ptr frictions_;
    /** \brief The mean vector {r, g, b, friction} */
    Eigen::Vector4f mean_;
    /** \brief The covariance matrix {r, g, b, friction} */
    Eigen::Matrix4f covariance_;
};

#endif /* SUPERVOXEL_H_ */
