/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_SLAM_DUNK_H
#define SLAM_DUNK_SLAM_DUNK_H

#include "slamdunk/slamdunk_defines.h"
#include "slamdunk/camera_tracker.h"
#include <g2o/core/hyper_dijkstra.h>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>
#include <Eigen/StdVector>

#ifdef SLAMDUNK_ICP_ENABLED
#include <flann/flann.hpp>
#endif // SLAMDUNK_ICP_ENABLED

namespace slamdunk
{

struct SLAM_DUNK_API SlamDunkParams
{
  CameraTracker::Ptr tracker;
  unsigned min_extracted_features, rba_rings, icp_samples, icp_maxit;
  int cores;
  float icp_distance_th, icp_normal_th;
  float kf_overlapping; // [0,1], the lower it is, the more distant kfs are
  bool doicp, try_loop_inference;
  bool verbose, debug;

  // default values (you may want to create the tracker then)
  SlamDunkParams()
      : min_extracted_features(30u),
        rba_rings(3), icp_samples(1000), icp_maxit(50u),
        cores(4), icp_distance_th(0.2), icp_normal_th(30),
        kf_overlapping(0.5f), doicp(false), try_loop_inference(false), verbose(false), debug(false) {}
};

class SLAM_DUNK_API SlamDunk
{
public:
  enum // return states
  {
    TRACKING_FAILED = 0,
    FRAME_TRACKED,
    KEYFRAME_DETECTED
  };

public:
  const Eigen::Matrix3f m_inverse_kcam;

  SlamDunk(const Eigen::Matrix3f &inverse_kcam, const SlamDunkParams &params = SlamDunkParams())
      : m_inverse_kcam(inverse_kcam), m_tracker(params.tracker),
        m_min_extracted_features((int)params.min_extracted_features),
        m_kf_overlapping(params.kf_overlapping), m_processed_frames(0),
        m_rba_rings(params.rba_rings), m_icp_samples(params.icp_samples),
        m_icp_maxit(params.icp_maxit), m_cores(params.cores),
        m_icp_cosnormal_th(std::cos(M_PI * params.icp_normal_th / 180.f)),
        m_last_wmchi2(-1.), m_doicp(params.doicp), m_try_loop_inference(params.try_loop_inference),
        m_verbose(params.verbose), m_debug(params.debug)
  {
  }

  typedef std::vector<std::pair<double, Eigen::Isometry3d>,
                      Eigen::aligned_allocator<std::pair<double, Eigen::Isometry3d>>>
      StampedPoseVector;
  typedef boost::function<void(const StampedPoseVector &poses)> debugging_function;

  /** \brief The debugging function is called whenever a new keyframe is detected */
  inline void setDebuggingFunction(const debugging_function &f) { m_deb_func = f; }
  inline void unsetDebuggingFunction() { m_deb_func.clear(); }

  int operator()(const RGBDFrame &frame, Eigen::Isometry3d &estimated_pose);
  int operator()(const RGBDFrame &frame)
  {
    Eigen::Isometry3d fake;
    return (*this)(frame, fake);
  }

  void forceGlobalOptimization();

  void getMappedPoses(StampedPoseVector &poses) const;
  const StampedPoseVector &getMovedFrames() const { return m_moved_frames; }
  /// A wrapper to access frame data (thread safe)
  const FrameData *getFrameData(int id);
  /// A wrapper to get the residual from the last optimization (thread safe)
  double lastWeightedMeanChi2();
  unsigned getNumberOfKeyframes();

private:
#ifdef SLAMDUNK_ICP_ENABLED
  typedef flann::Index<flann::L2<float>> FLANNIndex;
  typedef boost::shared_ptr<flann::Index<flann::L2<float>>> FLANNIndexPtr;

  void updateICPConstraints(int ref_frame_id, float icp_sqDistance_th,
                            const std::vector<std::pair<int, FLANNIndexPtr>> &trees,
                            std::list<g2o::SparseOptimizer::Edge *> &icp_edges);
  void samplePointsAndNormals(const RGBDFrame &frame, FrameData &frameData) const;
#endif // SLAMDUNK_ICP_ENABLED

  CameraTracker::Ptr m_tracker;
  GraphBackend::Ptr m_graph_ptr;
  boost::scoped_ptr<g2o::HyperDijkstra> m_dijkstra;
  StampedPoseVector m_moved_frames;
  int m_min_extracted_features; // will be compared to cv::Mat::rows which is int
  float m_kf_overlapping;
  unsigned m_processed_frames, m_rba_rings, m_icp_samples, m_icp_maxit;
  int m_cores;
  float m_icp_cosnormal_th;
  double m_last_wmchi2;
  bool m_doicp, m_try_loop_inference;
  bool m_verbose, m_debug;
  debugging_function m_deb_func;
};
}

#endif // SLAM_DUNK_SLAM_DUNK_H
