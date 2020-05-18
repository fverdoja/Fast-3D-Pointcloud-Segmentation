/*
 * Copyright (c) 2014, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * * Adopted and modified on: 27/4/20
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi
 */

// #include <material_segmentation/GaussianMixture.h>
// #include <material_segmentation/Gaussian.h>
#include "gmm.h"

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// STL
#include <iostream>
#include <string>
#include <deque>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#ifndef GMM_GMR_H
#define GMM_GMR_H

// minimum number of gaussians
#define PARAM_NAME_GAUSSIAN_COUNT_MIN    "gaussian_count_min"
#define PARAM_DEFAULT_GAUSSIAN_COUNT_MIN 1

// search will terminate when the gaussian count reaches this OR...
#define PARAM_NAME_GAUSSIAN_COUNT_MAX    "gaussian_count_max"
#define PARAM_DEFAULT_GAUSSIAN_COUNT_MAX 3
// ...when the current BIC - lower BIC is higher than this
#define PARAM_NAME_BIC_TERM_THRESHOLD    "bic_termination_threshold"
#define PARAM_DEFAULT_BIC_TERM_THRESHOLD (double(1000000.0))

// the algorithm will initialize the EM model by splitting this column
#define PARAM_NAME_TIME_COLUMN_ID        "time_column_id"
#define PARAM_DEFAULT_TIME_COLUMN_ID     0

// decrease this to get more gaussians, increase to get less
#define PARAM_NAME_BIC_PARAMS_WEIGHT     "bic_params_weight"
#define PARAM_DEFAULT_BIC_PARAMS_WEIGHT  1.0

// input topics
// this is a Float32MultiArray
// with two dimensions, rows and columns
// each row contains a sample, each column a dimension
#define PARAM_NAME_DATA_INPUT_TOPIC      "data_input_topic"
#define PARAM_DEFAULT_DATA_INPUT_TOPIC   "/gmm/data"

// output topics
// outputs a gaussian_mixture_model/GaussianMixture.msg
#define PARAM_NAME_MIX_OUTPUT_TOPIC      "mix_output_topic"
#define PARAM_DEFAULT_MIX_OUTPUT_TOPIC   "/gmm/mix"

#endif // GMM_NODE_H
class GMM_GMR
{

public:
  // GMM_GMR();
  typedef unsigned int uint;
  std::tuple<std::vector<Eigen::VectorXf>,std::vector<Eigen::MatrixXf>,std::vector<float>> fit_gmm(const Eigen::MatrixXf & rosdata); 
  Eigen::VectorXf gaussPDF(const Eigen::VectorXf & mean, const Eigen::MatrixXf & cov,const Eigen::MatrixXf & data) const;
  std::tuple<Eigen::VectorXf,Eigen::MatrixXf> gmr(std::vector<float> & weights,std::vector<Eigen::VectorXf> &means, std::vector<Eigen::MatrixXf> &covariances, Eigen::MatrixXf x,int in_dim, int outdim);
  Eigen::MatrixXf VStack(const std::vector<Eigen::MatrixXf> &mat_vec);
  // void shutdownWaitingThread(); 
  // GMMNode(ros::NodeHandle & nh): m_nh(nh), m_shutting_down_thread(&GMMNode::shutdownWaitingThread,this)
  // {
  //   int temp_int;
  //   std::string temp_string;
  //   double temp_double;

  //   m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MAX,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MAX);
  //   m_gaussian_count_max = (temp_int > 0) ? temp_int : 1;

  //   m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MIN,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MIN);
  //   m_gaussian_count_min = (temp_int > 0) ? temp_int : 1;

  //   m_nh.param<std::string>(PARAM_NAME_DATA_INPUT_TOPIC,temp_string,PARAM_DEFAULT_DATA_INPUT_TOPIC);
  //   m_data_subscriber = m_nh.subscribe(temp_string,10,&GMMNode::queue_data,this);

  //   m_nh.param<double>(PARAM_NAME_BIC_TERM_THRESHOLD,temp_double,PARAM_DEFAULT_BIC_TERM_THRESHOLD);
  //   m_bic_termination_threshold = temp_double;

  //   m_nh.param<int>(PARAM_NAME_TIME_COLUMN_ID,temp_int,PARAM_DEFAULT_TIME_COLUMN_ID);
  //   m_time_column_id = temp_int >= 0 ? temp_int : PARAM_DEFAULT_TIME_COLUMN_ID;

  //   m_nh.param<double>(PARAM_NAME_BIC_PARAMS_WEIGHT,temp_double,PARAM_DEFAULT_BIC_PARAMS_WEIGHT);
  //   m_bic_params_weight = temp_double;

  //   m_nh.param<std::string>(PARAM_NAME_MIX_OUTPUT_TOPIC,temp_string,PARAM_DEFAULT_MIX_OUTPUT_TOPIC);
  //   m_mix_publisher = m_nh.advertise<material_segmentation::GaussianMixture>(temp_string,2);
  // }

  class TerminationHandler: public GMMExpectationMaximization::ITerminationHandler
  {
    public:
    bool isTerminated() {return 0;}
  };

private:
  // ros::NodeHandle & m_nh;

  boost::mutex m_queue_mutex;
  boost::condition_variable m_queue_cond;

  uint m_gaussian_count_max = 3;
  uint m_gaussian_count_min = 1;

  uint m_time_column_id = 0;

  float m_bic_termination_threshold = 1000000.0;
  float m_bic_params_weight = 1.0;

  // ros::Subscriber m_data_subscriber;
  // ros::Publisher m_mix_publisher;

  // this thread will simply wait for shutdown
  // and unlock all the conditions variables
  boost::thread m_shutting_down_thread;
};