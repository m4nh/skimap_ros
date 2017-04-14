/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_TRANSFORMATION_ESTIMATION_H
#define SLAM_DUNK_TRANSFORMATION_ESTIMATION_H

#include "slamdunk/slamdunk_defines.h"

#include <ctime>
#include <vector>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace slamdunk
{

void SLAM_DUNK_API estimateTransformationSVD(const std::vector<Eigen::Vector3f> &reference,
                                             const std::vector<Eigen::Vector3f> &query,
                                             const std::vector<std::pair<unsigned, unsigned>> &ref_query_ids,
                                             Eigen::Isometry3d &queryToRef);

class SLAM_DUNK_API SampleConsensus
{
public:
  typedef boost::shared_ptr<SampleConsensus> Ptr;
  typedef boost::shared_ptr<const SampleConsensus> ConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SampleConsensus(double threshold = 0.05)
      : m_max_iterations(1000), m_squared_threshold(threshold * threshold),
        m_probability(0.99), m_query2ref(Eigen::Isometry3d::Identity()), m_verbose(false) {}
  virtual ~SampleConsensus() {}

  inline void setVerbose(bool verbose) { m_verbose = verbose; }
  inline bool isVerbose() const { return m_verbose; }

  inline void setMaxIterations(unsigned max_iterations) { m_max_iterations = max_iterations; }
  inline unsigned getMaxIterations() const { return m_max_iterations; }
  inline void setDistanceThreshold(double threshold) { m_squared_threshold = threshold * threshold; }
  inline double getSquaredDistanceThreshold() const { return m_squared_threshold; }

  /** \brief Set the desired probability of choosing at least one sample free from outliers. */
  inline void setProbability(double probability)
  {
    if (probability < 1 && probability >= 0)
      m_probability = probability;
  }
  inline double getProbability() const { return m_probability; }

  virtual bool findInliers(const std::vector<Eigen::Vector3f> &reference,
                           const std::vector<Eigen::Vector3f> &query,
                           const std::vector<std::pair<unsigned, unsigned>> &ref_query_ids) = 0;

  inline const std::vector<unsigned> &getRefQueryInliers() { return m_inlier_indices; }
  /** \brief Get the best Query->Reference transformation */
  inline const Eigen::Isometry3d &queryToRef() { return m_query2ref; }

protected:
  unsigned m_max_iterations;
  double m_squared_threshold;
  double m_probability;
  std::vector<unsigned> m_inlier_indices;
  Eigen::Isometry3d m_query2ref;
  bool m_verbose;
};

class SLAM_DUNK_API RANSAC : public SampleConsensus
{
public:
  RANSAC(bool deterministic, float threshold = 0.05)
      : SampleConsensus(threshold), m_deterministic(deterministic)
  {
    if (!m_deterministic)
      m_rnd_eng.seed(static_cast<unsigned>(std::time(NULL)));
  }

  virtual bool findInliers(const std::vector<Eigen::Vector3f> &reference,
                           const std::vector<Eigen::Vector3f> &query,
                           const std::vector<std::pair<unsigned, unsigned>> &ref_query_ids);

protected:
  boost::mt19937 m_rnd_eng;
  bool m_deterministic;
};
}

#endif // SLAM_DUNK_TRANSFORMATION_ESTIMATION_H
