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
 *
 *  * Adopted and modified on: 27/4/20
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi
 */

#ifndef GMM_H
#define GMM_H

// Eigen
#include <Eigen/Dense>

// boost
#include <boost/shared_ptr.hpp>

// STL
#include <vector>
#include <cmath>

class GMMExpectationMaximization
{
  public:
  typedef unsigned int uint;
  typedef Eigen::VectorXf VectorX;
  typedef std::vector<VectorX> VectorXVector;
  typedef Eigen::MatrixXf MatrixX;
  typedef std::vector<MatrixX> MatrixXVector;
  typedef float Real;
  typedef std::vector<Real> RealVector;
  typedef boost::shared_ptr<GMMExpectationMaximization> Ptr;

  class ITerminationHandler
  {
    public:
    typedef boost::shared_ptr<ITerminationHandler> Ptr;
    virtual bool isTerminated() = 0;

    virtual ~ITerminationHandler() {}
  };

  GMMExpectationMaximization();

  void setMaxIterations(uint n) {m_max_iterations = n; }
  uint getMaxIterations() {return m_max_iterations; }

  void setTerminationHandler(ITerminationHandler::Ptr th) {m_termination_handler = th; }

  void setEpsilon(Real e) {m_epsilon = e; }
  Real getEpsilon() {return m_epsilon; }

  // the algorithm stops when an iteration increases log-likelyhood less than this 
  void setTerminationThreshold(Real t) {m_termination_threshold = t; }
  Real getTerminationThreshold() {return m_termination_threshold; }

  /// use this version of execute to just execute the algorithm, with automatic initialization
  /// @param data a RxC matrix, where C is the dimension of the problem and R the number of samples
  /// @returns the number of iterations performed (0 if error)
  uint execute(uint num_gaussians,uint time_column,const MatrixX & data);

  // initialize by splitting data in equal intervals of the column with id col
  // false if error
  bool autoInitializeByEqualIntervals(uint num_gaussians,uint col,const MatrixX & data);
  /// execute the algorithm on the data
  /// @param data the data
  /// @returns the number of iterations performed (0 if error)
  uint execute(const MatrixX & data);

  // index that shows the goodness of the current mixture to represent the data
  // the lower the better
  Real getBIC(const MatrixX & data) const;
  void setBicParamsWeight(Real imp) {m_bic_params_weight = imp; }

  const VectorXVector & getMeans() const {return m_means; }
  const MatrixXVector & getCovariances() const {return m_covs; }
  const RealVector & getWeights() const {return m_weights; }

  // manual initialization
  // all these vectors MUST have the same size, when execute is called
  void setMeans(const VectorXVector & means) {m_means = means; }
  void setCovariances(const MatrixXVector & covariances) {m_covs = covariances; }
  void setWeights(const RealVector & weights) {m_weights = weights; }

  Real gauss(const VectorX & mean,const MatrixX & cov,const VectorX & pt) const;
  Real expectation(const VectorX & pt) const;

  private:
  uint m_max_iterations;
  Real m_epsilon;
  Real m_termination_threshold;
  Real m_bic_params_weight;

  VectorXVector m_means;
  MatrixXVector m_covs;
  RealVector m_weights;

  ITerminationHandler::Ptr m_termination_handler;
};

#endif // GMM_H
