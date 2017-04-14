/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "slamdunk/feature_matcher.h"
#include <list>

// Ctors
slamdunk::OpenCVFlannRatioMatcher::OpenCVFlannRatioMatcher(bool interModelRatio, float ratio, int cores)
    : OpenCVRatioMatcher(interModelRatio, ratio, new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(), new cv::flann::SearchParams()), cores)
//: OpenCVRatioMatcher(interModelRatio, ratio, new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(1), new cv::flann::SearchParams(-1)), cores) // Deterministic
//: OpenCVRatioMatcher(interModelRatio, ratio, new cv::BFMatcher(cv::NORM_L2, false), cores) // Brute Force
{
}

slamdunk::OpenCVLSHRatioMatcher::OpenCVLSHRatioMatcher(bool interModelRatio, float ratio, int cores)
    //: OpenCVRatioMatcher(interModelRatio, ratio, new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(16, 10, 2), new cv::flann::SearchParams()), cores)
    : OpenCVRatioMatcher(interModelRatio, ratio, new cv::BFMatcher(cv::NORM_HAMMING, false), cores) // Brute Force
{
}

void slamdunk::OpenCVRatioMatcher::setModels(const std::vector<cv::Mat> &models)
{
  assert(m_matcher);

  m_matcher->clear();
  m_matcher->add(models);
  m_matcher->train();

  m_model_zeroes.resize(models.size());
  int nDesc = 0;
  for (unsigned i = 0; i < models.size(); ++i)
  {
    m_model_zeroes[i] = nDesc;
    nDesc += models[i].rows;
  }

  m_model_indices.clear();
  m_model_indices.reserve(nDesc);
  for (unsigned i = 0; i < models.size(); ++i)
  {
    m_model_indices.insert(m_model_indices.end(), (size_t)models[i].rows, i);
  }
  assert((int)m_model_indices.size() == nDesc);
}

void slamdunk::OpenCVRatioMatcher::match(cv::Mat query, std::vector<FMatch> &matches)
{
  assert(m_matcher);
  std::vector<std::vector<cv::DMatch>> nn_matches;

  m_matcher->knnMatch(query, nn_matches, SLAM_DUNK_KNN_NEIGHBORS);

  std::list<cv::DMatch> match_list;
  for (int k = 0; k < query.rows; ++k)
  {
    const std::vector<cv::DMatch> &q_matches = nn_matches[k];
    if (q_matches.size() < 2)
    {
      continue;
    }

    const cv::DMatch &first_match = q_matches[0];
    const int model1 = first_match.imgIdx;

    // find 2nd nearest neighbor descriptor
    for (size_t neigh_idx = 1; neigh_idx < q_matches.size(); ++neigh_idx)
    {
      const cv::DMatch &current_match = q_matches[neigh_idx];
      assert(first_match.distance <= current_match.distance);

      // no need to check the distance because results are sorted
      if ((current_match.imgIdx != model1 && m_inter_model_ratio) ||
          (current_match.imgIdx == model1 && !m_inter_model_ratio))
      {
        if (first_match.distance <= m_ratio2 * current_match.distance && current_match.distance > 0)
        {
          cv::DMatch ratio_match = first_match;
          ratio_match.distance = first_match.distance / current_match.distance;
          match_list.push_back(ratio_match);
        }
        break;
      }
    }
  }

  match_list.sort();

  // clear double matches
  std::vector<bool> mask(m_model_indices.size(), true);
  matches.clear();
  matches.reserve(match_list.size());

  for (std::list<cv::DMatch>::const_iterator it = match_list.begin(); it != match_list.end(); ++it)
  {
    if (mask[m_model_zeroes[it->imgIdx] + it->trainIdx])
    {
      mask[m_model_zeroes[it->imgIdx] + it->trainIdx] = false;

      FMatch fm;
      fm.m_model_idx = it->imgIdx;
      fm.m_feat_idx = it->trainIdx;
      fm.m_query_idx = it->queryIdx;
      fm.m_match_score = 1.f - std::sqrt(it->distance);
      matches.push_back(fm);
    }
  }
}