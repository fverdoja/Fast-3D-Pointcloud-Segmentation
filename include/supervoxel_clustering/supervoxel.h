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

/**
 * Data structure representing a supervoxel or segmentation region. It extends
 * the standard pcl::Supervoxel by adding friction and variances for all the 
 * region properties (i.e., friction, color, position, and normal)
 */
template <typename PointT>
class Supervoxel : public pcl::Supervoxel<PointT> {
  public: 
    Supervoxel() : 
      pcl::Supervoxel<PointT>(),
      frictions_(new pcl::PointCloud<pcl::PointXYZI> ()),
      mean_(4),
      covariance_(4,4)
      {}
    Supervoxel(const pcl::Supervoxel<PointT>& s) : 
      pcl::Supervoxel<PointT>(s),
      frictions_(new pcl::PointCloud<pcl::PointXYZI> ()),
      mean_(4),
      covariance_(4,4)
      {}
    
    typedef boost::shared_ptr<Supervoxel<PointT> > Ptr;
    typedef boost::shared_ptr<const Supervoxel<PointT> > ConstPtr;

    /** \brief Computes the mean vector and covariance matrix */
    void compute_statistics() {
      // Mean vector
      mean_ << this->centroid_.r,
               this->centroid_.g,
               this->centroid_.b,
               this->friction_;

      // Covariance matrix
      float rr, rg, rb, rf, gg, gb, gf, bb, bf, ff = 0;
      int n_voxels = this->voxels_->size() - 1;       // N-1 for unbias estimation
      int n_frictions = this->frictions_->size() - 1; // N-1 for unbias estimation

      if(n_voxels > 0) {
        for (auto const &iter : this->voxels_->points) {
          float r_ = iter.r - mean_(0);
          float g_ = iter.g - mean_(1);
          float b_ = iter.b - mean_(2);
          rr += (r_ * r_) / n_voxels;
          gg += (g_ * g_) / n_voxels;
          bb += (b_ * b_) / n_voxels;
          rg += (r_ * g_) / n_voxels;
          rb += (r_ * b_) / n_voxels;
          gb += (g_ * b_) / n_voxels;
        }
      } else {
        rr, bb, gg = 1;
      }
      
      if(n_frictions > 0) {
        std::vector<int> nn_id(1);
        std::vector<float> nn_squared_dist(1);
        pcl::KdTreeFLANN<PointT> kdtree;
      
        kdtree.setInputCloud(this->voxels_);

        for(auto const &iter : this->frictions_->points) {
          PointT p;
          p.x = iter.x;
          p.y = iter.y;
          p.z = iter.z;
          if(kdtree.nearestKSearch(p, 1, nn_id, nn_squared_dist) > 0) {
            for(std::size_t i = 0; i < nn_id.size (); ++i) {
              PointT nn_p = this->voxels_->points[ nn_id[i] ];
              float r_ = nn_p.r - mean_(0);
              float g_ = nn_p.g - mean_(1);
              float b_ = nn_p.b - mean_(2);
              float f_ = iter.intensity - mean_(3);
              rf += (r_ * f_) / n_frictions;
              gf += (g_ * f_) / n_frictions;
              bf += (b_ * f_) / n_frictions;
              ff += (f_ * f_) / n_frictions;
            }
          } else {
            std::cout << "WARNING match not found!!!" << std::endl;
          }
        }
      } else {
        ff = 1;
      }
      
      covariance_ << rr, rg, rb, rf, 
                     rg, gg, gb, gf,
                     rb, gb, bb, bf,
                     rf, gf, bf, ff;

      if(n_frictions > 0) {
        std::cout << "*********" << n_voxels << " - " << n_frictions << std::endl;
        std::cout << covariance_ << std::endl << std::endl;
      }
    }

    /** \brief The friction calculated for the voxels contained in the supervoxel */
    float friction_;
    /** \brief A Pointcloud of the voxels along the haptic track in the supervoxel */
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr frictions_;
    /** \brief The mean vector {r, g, b, friction} */
    Eigen::VectorXf mean_;
    /** \brief The covariance matrix {r, g, b, friction} */
    Eigen::MatrixXf covariance_;
};

#endif /* SUPERVOXEL_H_ */
