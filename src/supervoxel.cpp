/*
 * supervoxel.cpp
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

#include "supervoxel_clustering/supervoxel.h"

void Supervoxel::compute_statistics() {
  // Mean vector
  mean_ << this->centroid_.r,
           this->centroid_.g,
           this->centroid_.b,
           this->friction_;

  // Covariance matrix
  float rr, rg, rb, rf, gg, gb, gf, bb, bf, ff;
  rr = rg = rb = rf = gg = gb = gf = bb = bf = ff = 0;
  int n_voxels = this->voxels_->size();
  int n_frictions = this->frictions_->size();
  typename pcl::PointCloud<PointT>::iterator v_itr, v_itr_end;
  typename pcl::PointCloud<pcl::PointXYZI>::iterator f_itr, f_itr_end;

  if(n_voxels > 1) {
    v_itr = this->voxels_->begin();
    v_itr_end = this->voxels_->end();
    for (; v_itr != v_itr_end; ++v_itr) {
        float r = v_itr->r - mean_(0);
        float g = v_itr->g - mean_(1);
        float b = v_itr->b - mean_(2);
        rr = rr + (r * r) / (n_voxels - 1);
        gg = gg + (g * g) / (n_voxels - 1);
        bb = bb + (b * b) / (n_voxels - 1);
        rg = rg + (r * g) / (n_voxels - 1);
        rb = rb + (r * b) / (n_voxels - 1);
        gb = gb + (g * b) / (n_voxels - 1);
    }
  } else {
    rr = gg = bb = 1;
  }

  if(n_frictions > 1) {
    std::vector<int> nn_id(1);
    std::vector<float> nn_squared_dist(1);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(this->voxels_);

    f_itr = this->frictions_->begin();
    f_itr_end = this->frictions_->end();
    for (; f_itr != f_itr_end; ++f_itr) {
      PointT p;
      p.x = f_itr->x;
      p.y = f_itr->y;
      p.z = f_itr->z;
      if(kdtree.nearestKSearch(p, 1, nn_id, nn_squared_dist) > 0) {
        for(std::size_t i = 0; i < nn_id.size (); ++i) {
          PointT nn_p = this->voxels_->points[ nn_id[i] ];
          float r = nn_p.r - mean_(0);
          float g = nn_p.g - mean_(1);
          float b = nn_p.b - mean_(2);
          float f = f_itr->intensity - mean_(3);
          rf = rf + (r * f) / (n_frictions - 1);
          gf = gf + (g * f) / (n_frictions - 1);
          bf = bf + (b * f) / (n_frictions - 1);
          // rf = 1;
          // gf = 1;
          // bf = 1;
          ff = ff + (f * f) / (n_frictions - 1);
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

  friction_variance_ = ff;

  if(n_frictions > 0) {
    std::cout << "*********" << n_voxels << " - " << n_frictions << std::endl;
    std::cout << "mean: " << mean_.transpose() << std::endl << std::endl;
    std::cout << "cov: " << covariance_ << std::endl << std::endl;
  }
}