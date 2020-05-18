#include <supervoxel_clustering/gmm_gmr.h>

std::tuple<std::vector<Eigen::VectorXf>,std::vector<Eigen::MatrixXf>,std::vector<float>> GMM_GMR::fit_gmm(const Eigen::MatrixXf & rosdata)
{
  
  std::string BIC_condition;
  std::cout << "Do you want to you BIC to choose the best number of Gaussian components? (Type yes or no): ";
  std::getline(std::cin, BIC_condition);
    //Define variable
    const uint dim = rosdata.outerSize();
    const uint ndata = rosdata.innerSize();
    const uint size = rosdata.size();
    Eigen::MatrixXf eigendata(ndata,dim);
    eigendata = rosdata;
    uint best_num_gauss = 0;
    float best_bic = 0.0;
    GMMExpectationMaximization::Ptr best_g;
    // execute and find the best (lower) bic
    if (BIC_condition == "yes")
    {
      for (uint num_gauss = m_gaussian_count_min; num_gauss <= m_gaussian_count_max; num_gauss++)
      {
        printf("gmm: attempting gaussian count: %u \n",num_gauss);
        if (num_gauss > ndata)
        {
          printf("gmm: too few points (%u) to fit %u gaussian(s).",ndata,num_gauss);
          break;
        }

        GMMExpectationMaximization::Ptr gmix(new GMMExpectationMaximization);
        gmix->setTerminationHandler(GMMExpectationMaximization::ITerminationHandler::Ptr(new TerminationHandler));

        gmix->setBicParamsWeight(m_bic_params_weight);
        if (!gmix->execute(num_gauss,m_time_column_id,eigendata))
        {
          printf("gmm: EM failed.");
          continue;
        }

        float bic = gmix->getBIC(eigendata);
        printf("gmm: bic is: %f\n",bic);
        if (best_num_gauss == 0 || bic < best_bic)
        {
          best_bic = bic;
          best_num_gauss = num_gauss;
          best_g = gmix;
        }

        // the bic is rising: exit
        if (bic - best_bic > m_bic_termination_threshold)
          break;
      }

      if (!best_g)
      {
        printf("gmm: couldn't find any GMM.");
      }

      printf("gmm: chosen gaussian count %u\n",best_num_gauss);
    }
    else if (BIC_condition == "no")
    {
      std::string num_gauss_string;
      std::cout << "Specify the number of Gaussian components! (Type number): ";
      std::getline(std::cin, num_gauss_string);
      int num_gauss = stoi(num_gauss_string);
      printf("gmm: chosen gaussian count %u",num_gauss);
      GMMExpectationMaximization::Ptr gmix_specify(new GMMExpectationMaximization);
      gmix_specify->setTerminationHandler(GMMExpectationMaximization::ITerminationHandler::Ptr(new TerminationHandler));

      gmix_specify->setBicParamsWeight(m_bic_params_weight);
      if (!gmix_specify->execute(num_gauss,m_time_column_id,eigendata))
      {
        printf("gmm: EM failed.");
      }
      
      best_g = gmix_specify;
      best_num_gauss = num_gauss;
    }
    else {
      printf("Wrong input! Try again!");
    }
      // create the message and send
      const std::vector<Eigen::VectorXf> & means = best_g->getMeans();
      const std::vector<float> & weights = best_g->getWeights();
      const std::vector<Eigen::MatrixXf> & covariances = best_g->getCovariances();
      // data.reset();
      return std::tuple<std::vector<Eigen::VectorXf>,std::vector<Eigen::MatrixXf>,std::vector<float>>{means,covariances,weights};
}

Eigen::VectorXf GMM_GMR::gaussPDF(const Eigen::VectorXf & mean,
  const Eigen::MatrixXf & cov,const Eigen::MatrixXf & dataset) const
{
  float det = cov.determinant();
  uint dim = mean.size();
  auto vector_prob = Eigen::VectorXf(dataset.innerSize());
  // else, compute pdf
  Eigen::MatrixXf inverse_cov = cov.inverse();
  Eigen::VectorXf dist;
  for (size_t i = 0; i < dataset.innerSize(); i++)
  {
    Eigen::VectorXf datapoint = dataset.row(i);
    dist = datapoint - mean;
    float exp = - (dist.dot(inverse_cov * dist)) / 2.0;
    float den = std::sqrt(std::pow(2.0 * M_PI,dim) * std::abs(det));  
    float prob = std::exp(exp) / den;
    vector_prob[i] = prob;
  }
  return vector_prob;
}
  std::tuple<Eigen::VectorXf,Eigen::MatrixXf> GMM_GMR::gmr(std::vector<float> & weights,std::vector<Eigen::VectorXf> &means, std::vector<Eigen::MatrixXf> &covariances, Eigen::MatrixXf x,int in_dim, int outdim)
{
  int num_data = x.innerSize(); // number of data point of the input
  int num_GMMcomp = means.size(); // number of K GMM component
  int num_variable = means[0].innerSize(); //number of dimension of data 

  // Compute the influence of each GMM component, given input x
  auto Px = Eigen::ArrayXXf(num_data,num_GMMcomp);
  for (size_t i = 0; i < num_GMMcomp; i++)
  {
    Eigen::Ref<Eigen::VectorXf> mu = means[i].topLeftCorner(in_dim,1);
    Eigen::Ref<Eigen::MatrixXf> sigma = covariances[i].topLeftCorner(in_dim,in_dim);
    Px.matrix().col(i) = weights[i] * GMM_GMR::gaussPDF(mu,sigma,x);
  }
  auto sum_Px = Eigen::ArrayXXf(num_data,1);
  sum_Px = (Px.matrix().rowwise().sum()).replicate(1,num_GMMcomp); 
  Px = Px / sum_Px;

  // Compute expected means predicted_mean, given input x
  auto predicted_mean_temp = Eigen::ArrayXXf(num_data,num_GMMcomp);
  for (size_t i = 0; i < num_GMMcomp; i++)
  {
    auto muy_in = means[i].transpose().topLeftCorner(1,in_dim).replicate(num_data,1);
    auto muy_out = means[i].transpose().bottomRightCorner(1,outdim).replicate(num_data,1);
    auto sigma_out_in = covariances[i].bottomLeftCorner(outdim,in_dim);
    auto sigma_in_in = covariances[i].topLeftCorner(in_dim,in_dim);
    predicted_mean_temp.matrix().col(i) = muy_out + (x-muy_in)*((sigma_out_in * sigma_in_in.inverse()).transpose());
  }
  predicted_mean_temp = Px * predicted_mean_temp;
  Eigen::VectorXf predicted_mean = predicted_mean_temp.matrix().rowwise().sum();

  // Compute expected covariances predicted_covariances, given input x
  auto predicted_covariances_temp = Eigen::ArrayXXf(outdim,num_GMMcomp);
  for (size_t i = 0; i < num_GMMcomp; i++)
  {
    auto sigma_out_out = covariances[i].bottomRightCorner(outdim,outdim);
    auto sigma_out_in = covariances[i].bottomLeftCorner(outdim,in_dim);
    auto sigma_in_in = covariances[i].topLeftCorner(in_dim,in_dim);
    auto sigma_in_out = covariances[i].topRightCorner(in_dim,outdim);
    predicted_covariances_temp.matrix().col(i) = sigma_out_out - (sigma_out_in * sigma_in_in.inverse() * sigma_in_out);
  }
  auto temp =  predicted_covariances_temp.replicate(num_data,1);
  auto out_unsum = Px * temp;
  Eigen::MatrixXf predicted_covariances = out_unsum.rowwise().sum();
  return std::tuple<Eigen::VectorXf,Eigen::MatrixXf>{predicted_mean,predicted_covariances}; 
}

Eigen::MatrixXf GMM_GMR::VStack(const std::vector<Eigen::MatrixXf> &mat_vec) {
  assert(!mat_vec.empty());
  long num_cols = mat_vec[0].cols();
  size_t num_rows = 0;
  for (size_t mat_idx = 0; mat_idx < mat_vec.size(); ++mat_idx) {
    assert(mat_vec[mat_idx].cols() == num_cols);
    num_rows += mat_vec[mat_idx].rows();
  }
  Eigen::MatrixXf vstacked_mat(num_rows, num_cols);
  size_t row_offset = 0;
  for (size_t mat_idx = 0; mat_idx < mat_vec.size(); ++mat_idx) {
    long cur_rows = mat_vec[mat_idx].rows();
    vstacked_mat.middleRows(row_offset, cur_rows) = mat_vec[mat_idx];
    row_offset +=  cur_rows;
  }
  return vstacked_mat;
}