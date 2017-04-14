/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_DATA_H
#define SLAM_DUNK_DATA_H

#include "slamdunk/slamdunk_defines.h"
#include <opencv2/core/core.hpp>
#ifndef NDEBUG
#include <opencv2/features2d/features2d.hpp>
#endif
#include <g2o/core/optimizable_graph.h>

namespace slamdunk
{

struct SLAM_DUNK_API RGBDFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double m_timestamp;
  int secs;
  int nsecs;

  cv::Mat m_color_image;
  cv::Mat_<float> m_depth_image;
  cv::Mat_<unsigned short> m_depth_image_mm;

  const static Eigen::Matrix3f m_inverse_kcam;
};

RGBDFrame cloneRGBDFrame(const RGBDFrame &frame);

struct SLAM_DUNK_API FrameToFrameMatch
{
  int m_matching_frame_id;
  int m_matching_frame_feat, m_ref_frame_feat;
  float m_score;
};

class SLAM_DUNK_API FrameData : public g2o::OptimizableGraph::Data
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameData() {}
  FrameData(const FrameData &fd)
      : m_timestamp(fd.m_timestamp), m_descriptors(fd.m_descriptors.clone()),
        m_keypoints(fd.m_keypoints), m_samplesPts(fd.m_samplesPts), m_samplesNormals(fd.m_samplesNormals)
#ifndef NDEBUG
        ,
        m_kpts2d(fd.m_kpts2d), m_image(fd.m_image)
#endif
  {
  }

  FrameData &operator=(const FrameData &fd)
  {
    m_timestamp = fd.m_timestamp;
    fd.m_descriptors.copyTo(m_descriptors);
    m_keypoints = fd.m_keypoints;
    m_samplesPts = fd.m_samplesPts;
    m_samplesNormals = fd.m_samplesNormals;
/////////////////////////////////////////////////////////
#ifndef NDEBUG
    m_kpts2d = fd.m_kpts2d;
    m_image = fd.m_image;
#endif
    /////////////////////////////////////////////////////////
    return *this;
  }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  double m_timestamp;
  cv::Mat m_descriptors;
  std::vector<Eigen::Vector3f> m_keypoints;
  Eigen::VectorXf m_samplesPts;
  std::vector<Eigen::Vector3f> m_samplesNormals;
/////////////////////////////////////////////////////////
#ifndef NDEBUG
  // 2d kpts are used for debugging purpose only
  std::vector<cv::KeyPoint> m_kpts2d;
  cv::Mat m_image;
#endif
  /////////////////////////////////////////////////////////
};

SLAM_DUNK_API cv::Mat_<float> depthBilateralFilter(const cv::Mat_<float> &input, double sigmaDepth, double sigmaSpace);

struct SLAM_DUNK_API ObjectData
{
  std::string m_tag;
  int m_global_id;
  std::vector<Eigen::Vector3f> m_keypoints;
};
}

#endif // SLAM_DUNK_DATA_H
