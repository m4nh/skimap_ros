
#include "slamdunk/feature_matcher.h"
#include <list>

template<class FLANNDistanceTp>
void slamdunk::RatioMatcher<FLANNDistanceTp>::setModels(const std::vector<cv::Mat>& models)
{
  assert(cv::DataType<FLANNElementTp>::type == models[0].type());

  m_model_zeroes.resize(models.size());
  int nDesc = 0;
  for(unsigned i = 0; i < models.size(); ++i)
  {
    m_model_zeroes[i] = nDesc;
    nDesc += models[i].rows;
  }
  m_descriptors.create(nDesc, models[0].cols);
  m_model_indices.clear();
  m_model_indices.reserve(nDesc);
  for(unsigned i = 0; i < models.size(); ++i)
  {
    m_model_indices.insert(m_model_indices.end(), (size_t)models[i].rows, i);
    models[i].copyTo(m_descriptors.rowRange(m_model_zeroes[i], m_model_zeroes[i]+models[i].rows));
  }
  assert((int)m_model_indices.size() == nDesc);

  delete m_index;
  m_index = createIndex(flann::Matrix<FLANNElementTp>(m_descriptors[0], m_descriptors.rows, m_descriptors.cols));
  m_index->buildIndex();
}

namespace
{
  struct MatchHypothesis
  {
    int feat_gidx, query_idx;
    float ratio;
  };

  bool matchH_comparison(const MatchHypothesis& mh0, const MatchHypothesis& mh1)
  {
    return mh0.ratio < mh1.ratio;
  }
}

template<class FLANNDistanceTp>
void slamdunk::RatioMatcher<FLANNDistanceTp>::match(cv::Mat query, std::vector<FMatch>& matches)
{
  assert(cv::DataType<FLANNElementTp>::type == query.type());
  assert(query.cols == m_descriptors.cols);

  typedef typename FLANNDistanceTp::ResultType DistanceType;

  flann::SearchParams search_params(16, 0, true);
  //flann::SearchParams search_params(-1, 0, true);
  search_params.cores = m_cores;
  flann::Matrix<FLANNElementTp> query_mtx(query.ptr<FLANNElementTp>(0), query.rows, query.cols, query.step.p[0]);
  std::vector<std::vector<size_t> > indices(SLAM_DUNK_KNN_NEIGHBORS * query.rows);
  std::vector<std::vector<DistanceType> > dists(SLAM_DUNK_KNN_NEIGHBORS * query.rows);

  //flann::Matrix<int> indices(new int[SLAM_DUNK_KNN_NEIGHBORS * query.rows], query.rows, SLAM_DUNK_KNN_NEIGHBORS);
  //flann::Matrix<DistanceType> dists(new DistanceType[SLAM_DUNK_KNN_NEIGHBORS * query.rows], query.rows, SLAM_DUNK_KNN_NEIGHBORS);

  int n_matches = m_index->knnSearch(query_mtx, indices, dists, SLAM_DUNK_KNN_NEIGHBORS, search_params);

  std::list<MatchHypothesis> match_list;
  for(int k = 0; k < query.rows; ++k)
  {
    //const int* indexPtr = indices[k];
    //const DistanceType* distsPtr = dists[k];
	const std::vector<size_t> &indexPtr = indices[k];
	const std::vector<DistanceType> &distsPtr = dists[k];

	if(indexPtr.size() < 2)
	{
		continue;
	}

    const unsigned model1 = m_model_indices[ indexPtr[0] ];

    // find 2nd nearest neighbor descriptor
	for(unsigned ne = 1; ne < indexPtr.size(); ++ne)
    {
      assert(distsPtr[0] <= distsPtr[ne]);

      // no need to check the distance because results are sorted
      if((m_model_indices[ indexPtr[ne] ] != model1 && m_inter_model_ratio) ||
          (m_model_indices[ indexPtr[ne] ] == model1 && !m_inter_model_ratio))
      {
        if(distsPtr[0] <= m_ratio2 * distsPtr[ne] && distsPtr[ne] > 0)
        {
          MatchHypothesis mh;
          mh.feat_gidx = (int)indexPtr[0];
          mh.query_idx = k;
          mh.ratio = distsPtr[0] / (float)distsPtr[ne];
          match_list.push_back(mh);
        }
        break;
      }
    }
  }
  match_list.sort(matchH_comparison);

  // clear double matches
  std::vector<bool> mask(m_model_indices.size(), true);
  matches.clear();
  matches.reserve(match_list.size());
  for(std::list<MatchHypothesis>::const_iterator it = match_list.begin(); it != match_list.end(); ++it)
    if(mask[it->feat_gidx])
    {
      mask[it->feat_gidx] = false;
      FMatch fm;
      fm.m_model_idx = m_model_indices[ it->feat_gidx ];
      fm.m_feat_idx = it->feat_gidx - m_model_zeroes[fm.m_model_idx];
      fm.m_query_idx = it->query_idx;
      fm.m_match_score = 1.f - std::sqrt(it->ratio);
      matches.push_back(fm);
    }

  //delete[] indices.ptr();
  //delete[] dists.ptr();
}


template<class FLANNDistanceTp>
typename slamdunk::RatioMatcherKDTreeIndex<FLANNDistanceTp>::FLANNIndex*
  slamdunk::RatioMatcherKDTreeIndex<FLANNDistanceTp>::createIndex(const flann::Matrix<FLANNElementTp>& features) const
{
  return new FLANNIndex(features, flann::KDTreeIndexParams(4));
  //return new FLANNIndex(features, flann::KDTreeIndexParams(1));
}

template<class FLANNDistanceTp>
typename slamdunk::RatioMatcherLSHIndex<FLANNDistanceTp>::FLANNIndex* 
  slamdunk::RatioMatcherLSHIndex<FLANNDistanceTp>::createIndex(const flann::Matrix<FLANNElementTp>& features) const
{
  return new FLANNIndex(features, flann::LshIndexParams(1));
}
