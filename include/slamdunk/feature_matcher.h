
#ifndef SLAM_DUNK_FEATURE_MATCHER_H
#define SLAM_DUNK_FEATURE_MATCHER_H

#include "slamdunk/slamdunk_defines.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifdef SLAMDUNK_FLANN_ENABLED
  #include <flann/flann.hpp>
#endif // SLAMDUNK_FLANN_ENABLED

#include <boost/shared_ptr.hpp>

namespace slamdunk
{

  struct SLAM_DUNK_API FMatch
  {
    int m_model_idx, m_feat_idx, m_query_idx;
    float m_match_score;
  };

  class SLAM_DUNK_API FeatureMatcher
  {
    public:
      typedef boost::shared_ptr<FeatureMatcher> Ptr;
      typedef boost::shared_ptr<const FeatureMatcher> ConstPtr;

      virtual ~FeatureMatcher() {};
      virtual void setModels(const std::vector<cv::Mat>& models) = 0;
      virtual void match(cv::Mat query, std::vector<FMatch>& matches) = 0;
  };

#ifdef SLAMDUNK_FLANN_ENABLED
  template<class FLANNDistanceTp>
  class RatioMatcher : public FeatureMatcher
  {
    public:
      typedef typename FLANNDistanceTp::ElementType FLANNElementTp;
      typedef flann::Index<FLANNDistanceTp> FLANNIndex;

      RatioMatcher(bool interModelRatio, float ratio, int cores)
        : m_cores(cores), m_inter_model_ratio(interModelRatio), m_ratio2(ratio*ratio), m_index(NULL) {}

      virtual ~RatioMatcher() { delete m_index; }

      virtual void setModels(const std::vector<cv::Mat>& models);
      virtual void match(cv::Mat query, std::vector<FMatch>& matches);
      
      inline void setCores(int cores) { m_cores = cores;}
      inline int getCores() const { return m_cores; }

      /// If true, do the ratio among different models (eg obj detection), otherwise within the same model (eg frame tracking)
      inline void setInterModelRatio(bool onoff) { m_inter_model_ratio = onoff; }
      inline bool getInterModelRatio() const { return m_inter_model_ratio; }

      inline void setRatio(float ratio) { m_ratio2 = ratio*ratio; }
      inline float getSquaredRatio() const { return m_ratio2; }

    protected:
      virtual FLANNIndex* createIndex(const flann::Matrix<FLANNElementTp>& features) const = 0;

    private:
      enum { SLAM_DUNK_KNN_NEIGHBORS = 32 };

      int m_cores;
      bool m_inter_model_ratio;
      float m_ratio2;
      FLANNIndex* m_index;
      cv::Mat_<FLANNElementTp> m_descriptors;
      std::vector<unsigned> m_model_indices;
      std::vector<int> m_model_zeroes;
  };

  template<class FLANNDistanceTp>
  class RatioMatcherKDTreeIndex : public RatioMatcher<FLANNDistanceTp>
  {
    public:
      typedef typename RatioMatcher<FLANNDistanceTp>::FLANNElementTp FLANNElementTp;
      typedef typename RatioMatcher<FLANNDistanceTp>::FLANNIndex FLANNIndex;

      RatioMatcherKDTreeIndex(bool interModelRatio, float ratio = 0.8, int cores = 1)
        : RatioMatcher<FLANNDistanceTp>(interModelRatio, ratio, cores) {}
    protected:
      virtual FLANNIndex* createIndex(const flann::Matrix<FLANNElementTp>& features) const;
  };

  template<class FLANNDistanceTp>
  class RatioMatcherLSHIndex : public RatioMatcher<FLANNDistanceTp>
  {
    public:
      typedef typename RatioMatcher<FLANNDistanceTp>::FLANNElementTp FLANNElementTp;
      typedef typename RatioMatcher<FLANNDistanceTp>::FLANNIndex FLANNIndex;

	  RatioMatcherLSHIndex(bool interModelRatio, float ratio = 0.8, int cores = 1)
        : RatioMatcher<FLANNDistanceTp>(interModelRatio, ratio, cores) {}
    protected:
      virtual FLANNIndex* createIndex(const flann::Matrix<FLANNElementTp>& features) const;
  };

#endif // SLAMDUNK_FLANN_ENABLED

  class SLAM_DUNK_API OpenCVRatioMatcher : public FeatureMatcher
  {
    public:
      virtual void setModels(const std::vector<cv::Mat>& models);
      virtual void match(cv::Mat query, std::vector<FMatch>& matches);

      /// If true, do the ratio among different models (eg obj detection), otherwise within the same model (eg frame tracking)
      inline void setInterModelRatio(bool onoff) { m_inter_model_ratio = onoff; }
      inline bool getInterModelRatio() const { return m_inter_model_ratio; }

      inline void setRatio(float ratio) { m_ratio2 = ratio*ratio; }
      inline float getSquaredRatio() const { return m_ratio2; }

    protected:
      OpenCVRatioMatcher(bool interModelRatio, float ratio, cv::Ptr<cv::DescriptorMatcher> matcher, int cores = 1)
        : m_inter_model_ratio(interModelRatio), m_ratio2(ratio*ratio), m_matcher(matcher), m_cores(cores) {}

    private:
      enum { SLAM_DUNK_KNN_NEIGHBORS = 32 };

      int m_cores;
      bool m_inter_model_ratio;
      float m_ratio2;
      cv::Ptr<cv::DescriptorMatcher> m_matcher;

      std::vector<unsigned> m_model_indices;
      std::vector<int> m_model_zeroes;
  };

  class SLAM_DUNK_API OpenCVFlannRatioMatcher : public OpenCVRatioMatcher
  {
    public:
      OpenCVFlannRatioMatcher(bool interModelRatio, float ratio, int cores = 1);
  };

  class SLAM_DUNK_API OpenCVLSHRatioMatcher : public OpenCVRatioMatcher
  {
    public:
      OpenCVLSHRatioMatcher(bool interModelRatio, float ratio, int cores = 1);
  };

#ifdef SLAMDUNK_FLANN_ENABLED
  typedef RatioMatcherKDTreeIndex<flann::L2<float> > RatioMatcherL2;
  typedef RatioMatcherKDTreeIndex<flann::L2<double> > RatioMatcherL2D;
  //typedef RatioMatcherLSHIndex<flann::Hamming<unsigned char> > RatioMatcherHamming;
  typedef RatioMatcherLSHIndex<flann::HammingLUT> RatioMatcherHamming;
#else // SLAMDUNK_FLANN_ENABLED
  typedef OpenCVFlannRatioMatcher RatioMatcherL2;
  typedef OpenCVFlannRatioMatcher RatioMatcherL2D;
  typedef OpenCVLSHRatioMatcher RatioMatcherHamming;
#endif

}

#ifdef SLAMDUNK_FLANN_ENABLED
  #include "impl/ratio_matcher.hpp"
#endif // SLAMDUNK_FLANN_ENABLED

#endif // SLAM_DUNK_FEATURE_MATCHER_H
