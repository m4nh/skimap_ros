
#include "slamdunk/transformation_estimation.h"
#include "slamdunk/pretty_printer.h"

#include <iostream>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_int.hpp>
#include <Eigen/SVD>
#include <Eigen/LU> // required for determinant computation

void SLAM_DUNK_API slamdunk::estimateTransformationSVD(const std::vector<Eigen::Vector3f>& reference,
                                                const std::vector<Eigen::Vector3f>& query,
                                                const std::vector< std::pair<unsigned, unsigned> >& ref_query_ids,
                                                Eigen::Isometry3d& queryToRef)
{
  const unsigned no_of_matches = ref_query_ids.size();
  Eigen::Vector3f mean_ref(0,0,0), mean_qry(0,0,0);
  for(unsigned rqidx = 0; rqidx < no_of_matches; ++rqidx)
  {
    mean_ref += reference[ ref_query_ids[rqidx].first ];
    mean_qry += query[ ref_query_ids[rqidx].second ];
  }
  mean_ref /= (float)no_of_matches;
  mean_qry /= (float)no_of_matches;

  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for(unsigned rqidx = 0; rqidx < no_of_matches; ++rqidx)
  {
    H.noalias() += (query[ ref_query_ids[rqidx].second ] - mean_qry).cast<double>() *
                   (reference[ ref_query_ids[rqidx].first ] - mean_ref).cast<double>().transpose();
  }
  // SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = V * svd.matrixU().transpose();
  if(R.determinant() < 0.0)
  {
    V.col(2) = V.col(2)*-1.0;
    R.noalias() = V * svd.matrixU().transpose();
  }

  queryToRef.setIdentity();
  queryToRef.rotate(R);
  queryToRef.translation() = (mean_ref.cast<double>() - R*mean_qry.cast<double>());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
bool slamdunk::RANSAC::findInliers(const std::vector<Eigen::Vector3f>& reference,
                                const std::vector<Eigen::Vector3f>& query,
                                const std::vector< std::pair<unsigned, unsigned> >& ref_query_ids)
{
  if(ref_query_ids.size() < 3)
  {
    if(m_verbose)
      std::cout << SLAM_DUNK_WARNING_STR("Less than 3 matches provided") << std::endl;
    m_inlier_indices.clear();
    return false;
  }

  const unsigned no_of_matches = ref_query_ids.size();
  if(m_deterministic)
    m_rnd_eng.seed(42u);
  boost::variate_generator< boost::mt19937&, boost::uniform_int<unsigned> > rnd_gen(m_rnd_eng, boost::uniform_int<unsigned>(0, no_of_matches-1));
  unsigned current_max_iterations = m_max_iterations;

  unsigned a, b, c;
  Eigen::Matrix3d ref_mtx, qry_mtx;
  Eigen::Isometry3d current_transformation = Eigen::Isometry3d::Identity();
  unsigned best_score = 0;

  const double no_of_matches_inverse = 1.0/(double)no_of_matches;
  const double lognum = std::log(1-m_probability);

  unsigned it = 0;
  for(; it < current_max_iterations; ++it)
  {
    a = rnd_gen();
    do b = rnd_gen(); while(a == b);
    do c = rnd_gen(); while(c == b || c == a);

    ref_mtx.row(0) = reference[ ref_query_ids[a].first ].cast<double>();
    ref_mtx.row(1) = reference[ ref_query_ids[b].first ].cast<double>();
    ref_mtx.row(2) = reference[ ref_query_ids[c].first ].cast<double>();
    qry_mtx.col(0) = query[ ref_query_ids[a].second ].cast<double>();
    qry_mtx.col(1) = query[ ref_query_ids[b].second ].cast<double>();
    qry_mtx.col(2) = query[ ref_query_ids[c].second ].cast<double>();

    // compute centroids
    const Eigen::Vector3d cRef = (ref_mtx.row(0)+ref_mtx.row(1)+ref_mtx.row(2))*(0.33333333333333333333);
    const Eigen::Vector3d cQry = (qry_mtx.col(0)+qry_mtx.col(1)+qry_mtx.col(2))*(0.33333333333333333333);

    ref_mtx.row(0) -= cRef;
    ref_mtx.row(1) -= cRef;
    ref_mtx.row(2) -= cRef;
    qry_mtx.col(0) -= cQry;
    qry_mtx.col(1) -= cQry;
    qry_mtx.col(2) -= cQry;
    const Eigen::Matrix3d H = qry_mtx*ref_mtx;

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * svd.matrixU().transpose();
    if(R.determinant() < 0.0)
    {
      V.col(2) = V.col(2)*-1.0;
      R.noalias() = V * svd.matrixU().transpose();
    }
    current_transformation = Eigen::Isometry3d(R);
    current_transformation.translation() = cRef-R*cQry;

    // voting
    unsigned current_score = 0;
    for(unsigned rqidx = 0; rqidx < no_of_matches; ++rqidx)
      if((current_transformation * query[ ref_query_ids[rqidx].second ].cast<double>()
            - reference[ ref_query_ids[rqidx].first ].cast<double>()).squaredNorm() <= m_squared_threshold)
        ++current_score;  // TODO: weight the actual distance

    // update
    if(current_score > best_score)
    {
      best_score = current_score;
      m_query2ref = current_transformation;

      if(best_score == no_of_matches)
        current_max_iterations = 0;
      else
        current_max_iterations = std::min((double)m_max_iterations, std::ceil(lognum / std::log(1.0 - std::pow((double)best_score * no_of_matches_inverse, 3.0))));
    }
  }

  if(m_verbose)
    std::cout << SLAM_DUNK_INFO_STR("No of iterations: " << it) << std::endl;

  if(best_score < 4)
  {
    if(m_verbose)
      std::cout << SLAM_DUNK_WARNING_STR("Empty inlier set") << std::endl;
    m_inlier_indices.clear();
    return false;
  }

  // store indices
  m_inlier_indices.clear();
  m_inlier_indices.reserve(best_score);
  for(unsigned rqidx = 0; rqidx < no_of_matches; ++rqidx)
    if((m_query2ref * query[ ref_query_ids[rqidx].second ].cast<double>()
          - reference[ ref_query_ids[rqidx].first ].cast<double>()).squaredNorm() <= m_squared_threshold)
      m_inlier_indices.push_back(rqidx);

  return true;
}
