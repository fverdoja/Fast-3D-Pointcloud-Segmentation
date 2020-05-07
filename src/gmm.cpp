#include "supervoxel_clustering/gmm.h"

#define EPSILON (1e-5)
#define MAXITER 200
#define TERMINATION_THRESHOLD 1e-8f

GMMExpectationMaximization::GMMExpectationMaximization()
{
  m_max_iterations = MAXITER;
  m_epsilon = EPSILON;
  m_termination_threshold = TERMINATION_THRESHOLD;
  m_bic_params_weight = 1.0;
}

GMMExpectationMaximization::Real GMMExpectationMaximization::gauss(const VectorX & mean,
  const MatrixX & cov,const VectorX & pt) const
{
  Real det = cov.determinant();
  uint dim = mean.size();
  // check that the covariance matrix is invertible
  if (std::abs(det) < std::pow(m_epsilon,dim) * 0.1)
    return 0.0; // the gaussian has approximately zero width: the probability of any point falling into it is approximately 0.
 
  // else, compute pdf
  MatrixX inverse_cov = cov.inverse();
  VectorX dist = pt - mean;
  Real exp = - (dist.dot(inverse_cov * dist)) / 2.0;
  Real den = std::sqrt(std::pow(2.0 * M_PI,dim) * std::abs(det));
  return std::exp(exp) / den;
}

GMMExpectationMaximization::uint GMMExpectationMaximization::execute(uint num_gaussians,uint time_column,const MatrixX & data)
{
  if (!autoInitializeByEqualIntervals(num_gaussians,time_column,data))
    return 0;
  return execute(data);
}

GMMExpectationMaximization::uint GMMExpectationMaximization::execute(const MatrixX & dataset)
{  
  const uint data_count = dataset.rows();
  const uint num_gaussians = m_means.size();
  const uint dim = dataset.cols();

  MatrixX pxi(data_count,num_gaussians);
  MatrixX pix(data_count,num_gaussians);
  VectorX pxidatatot(data_count);
  VectorX weights(num_gaussians);
  VectorX ex(data_count);
  MatrixX ts(dim,dim);
  VectorX dif(dim);

  Real prev_log_likelyhood = 1.0;
  
  uint it_num;
  for (it_num = 0; it_num < m_max_iterations; it_num++)
  {
    for (uint g = 0; g < num_gaussians; g++)
      weights[g] = m_weights[g];

    for (uint d = 0; d < data_count; d++)
      for (uint g = 0; g < num_gaussians; g++)
        pxi(d,g) = gauss(m_means[g],m_covs[g],dataset.row(d).transpose());

    pxidatatot = pxi * weights;
    Real log_likelyhood = pxidatatot.array().log().sum() / Real(data_count);

    if (it_num != 0 && (std::abs(log_likelyhood / prev_log_likelyhood - 1.0) < m_termination_threshold))
      break;
    prev_log_likelyhood = log_likelyhood;

    for (uint d = 0; d < data_count; d++)
      pix.row(d) = (pxi.row(d).transpose().array() * weights.array()).transpose() / pxidatatot[d];
    
    ex = pix.colwise().sum();

    for(uint g = 0; g < num_gaussians; g++)
    {
      m_weights[g] = ex[g] / Real(data_count);

      m_means[g] = (dataset.transpose() * pix.col(g)) / ex[g];

      ts = MatrixX::Zero(dim,dim);
      for (uint d = 0; d < data_count; d++)
      {
        dif = dataset.row(d).transpose() - m_means[g];
        ts.noalias() += (dif * dif.transpose()) * pix(d,g);
      }
      m_covs[g] = (ts / ex[g]) + MatrixX::Identity(dim,dim) * m_epsilon;
    }

    // interruption point here
    if (m_termination_handler && m_termination_handler->isTerminated())
      return it_num;
  }

  return it_num;
}

bool GMMExpectationMaximization::autoInitializeByEqualIntervals(uint num_gaussians,uint col,const MatrixX & dataset)
{
  uint data_count = dataset.rows();
  uint dim = dataset.cols();

  if (!data_count || !dim)
    return false;

  std::vector<std::vector<uint> > index(num_gaussians);
  for(uint g = 0; g < num_gaussians; g++)
    index[g].reserve(data_count / num_gaussians);

  m_weights.clear();
  m_weights.resize(num_gaussians);
  m_means.clear();
  m_means.resize(num_gaussians,VectorX::Zero(dim));
  m_covs.clear();
  m_covs.resize(num_gaussians,MatrixX::Zero(dim,dim));

  // find max and min value for column col
  Real cmax = dataset(0,col);
  Real cmin = dataset(0,col);
  for(uint n = 1; n < data_count; n++)
  {
    if (dataset(n,col) > cmax) cmax = dataset(n,col);
    if (dataset(n,col) < cmin) cmin = dataset(n,col);
  }
  Real cspan = cmax - cmin;

  for(uint n = 0; n < data_count; n++) 
  {
    // compute gaussian index to which this point belongs
    uint gi = uint((dataset(n,col) - cmin) / (cspan + 1.0) * Real(num_gaussians));

    // sum the points to obtain means
    m_means[gi] += dataset.row(n);

    index[gi].push_back(n);
  }

  for (uint g = 0; g < num_gaussians; g++)
  {
    uint popsize = index[g].size();
    // avoid division by zero: if no samples are available, initialize to something from somewhere
    if (popsize == 0)
    {
      m_means[g] = dataset.row(g % data_count);
      m_covs[g] = MatrixX::Identity(dim,dim);
      m_weights[g] = 1.0f / Real(num_gaussians);
      continue;
    }

    // average by popsize
    m_means[g] /= Real(popsize);
    // same weight for all gaussians
    m_weights[g] = 1.0f / Real(num_gaussians);
     
    // compute covariance matrix
    for (uint p = 0; p < popsize; p++)
    {
      const Eigen::VectorXf & r = dataset.row(index[g][p]);
      const Eigen::VectorXf & m = m_means[g];
      m_covs[g] += (r - m) * (r - m).transpose();
    }

    m_covs[g] /= Real(popsize);
    m_covs[g] += MatrixX::Identity(dim,dim) * m_epsilon;
  }

  return true;
}

GMMExpectationMaximization::Real GMMExpectationMaximization::expectation(const VectorX & pt) const
{
  const uint num_gaussians = m_means.size();
  Real sum = 0;
  for (uint g = 0; g < num_gaussians; g++)
    sum += gauss(m_means[g],m_covs[g],pt) * m_weights[g];
  return sum;
}

GMMExpectationMaximization::Real GMMExpectationMaximization::getBIC(const MatrixX & dataset) const
{
  const uint dim = dataset.cols();
  const uint num_gaussians = m_means.size();

  Real number_of_parameters = (num_gaussians * dim * (dim + 1) / 2) + num_gaussians * dim + num_gaussians - 1;

  uint data_count = dataset.rows();
  Real sum = 0.0;
  
  for(uint i = 0; i < data_count; i++)
    sum += std::log(expectation(dataset.row(i).transpose()));

  return -sum + m_bic_params_weight * (number_of_parameters / 2.0) * std::log(Real(data_count));
}
