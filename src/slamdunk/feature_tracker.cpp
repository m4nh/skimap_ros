/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "slamdunk/camera_tracker.h"
#include "slamdunk/pretty_printer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>

#if defined(_MSC_VER) && _MSC_VER <= 1700 // VS 2012
#include <boost/math/special_functions.hpp>
using boost::math::isfinite;
#else
using std::isfinite;
#endif

#include "internal_timers.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::signalMovedFrames(const VertexPoseSet &vset)
{
  // update the tree
  if (vset.empty())
    return;

  bool ok = true;
  for (VertexPoseSet::const_iterator it = vset.begin(); it != vset.end(); ++it)
    ok &= m_tree.update(*it, Eigen::Vector2d((*it)->estimate().translation().x(), (*it)->estimate().translation().z()));

  if (!ok && m_verbose)
    std::cout << SLAM_DUNK_ERROR_STR("At least one frame has moved out of quad tree's bounds") << std::endl;

  // update the feature matcher
  calcActiveWindow();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::setFrames(const VertexPoseSet &vset)
{
  m_tree.clear();
  if (vset.empty())
  {
    m_reference_vxs.clear();
    return;
  }

  bool ok = true;
  for (VertexPoseSet::const_iterator it = vset.begin(); it != vset.end(); ++it)
    ok &= m_tree.insert(*it, Eigen::Vector2d((*it)->estimate().translation().x(), (*it)->estimate().translation().z()));

  if (!ok && m_verbose)
    std::cout << SLAM_DUNK_ERROR_STR("At least one frame is out of quad tree's bounds") << std::endl;

  // update the feature matcher
  calcActiveWindow();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::calcActiveWindow()
{
  {
    SLAM_DUNK_AUTO_CPU_TIMER("Active Window Update");

    QuadTreeType::ElementList elems;
    m_tree.query(m_last_tracked.m_pose.translation().x() - m_half_active_win_length,
                 m_last_tracked.m_pose.translation().z() - m_half_active_win_length,
                 2. * m_half_active_win_length, 2. * m_half_active_win_length, elems);
    m_active_win_center = Eigen::Vector2d(m_last_tracked.m_pose.translation().x(), m_last_tracked.m_pose.translation().z());
    m_active_win_pose = m_last_tracked.m_pose.cast<double>();

    m_reference_vxs.clear();
    std::vector<cv::Mat> frame_descriptors;

    Eigen::Vector4f homv(0, 0, 0, 1);
    for (QuadTreeType::ElementList::const_iterator it = elems.begin(); it != elems.end(); ++it)
    //if(m_last_tracked.m_pose.rotation().col(2).dot(it->m_data->estimate().rotation().col(2)) > m_min_pov_cosangle_th)
    //{
    //m_reference_vxs[frame_descriptors.size()] = it->m_data;
    //frame_descriptors.push_back(static_cast<const FrameData*>(it->m_data->userData())->m_descriptors);
    //}
    {
      const FrameData *fdata = static_cast<const FrameData *>(it->m_data->userData());
      if (m_frustum_feature_reduction)
      {
        // Count features within the frustum
        unsigned nfeats = 0;
        for (std::vector<Eigen::Vector3f>::const_iterator kpit = fdata->m_keypoints.begin(); kpit < fdata->m_keypoints.end(); ++kpit)
        {
          homv.head<3>() = *kpit;
          if (((m_frustum_mtx * homv).array() < 0).all())
            ++nfeats;
        }
        if ((float)nfeats / (float)fdata->m_keypoints.size() > m_perc_feat_overlap)
        {
          m_reference_vxs[frame_descriptors.size()] = it->m_data;
          frame_descriptors.push_back(fdata->m_descriptors);
        }
      }
      else
      {
        m_reference_vxs[frame_descriptors.size()] = it->m_data;
        frame_descriptors.push_back(fdata->m_descriptors);
      }
    }

    if (!frame_descriptors.empty())
      m_matcher->setModels(frame_descriptors);
    else
      std::cout << SLAM_DUNK_ERROR_STR("Active window is empty: using last frame") << std::endl;
  }

  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR("Active window updated with " << m_reference_vxs.size() << " frames") << std::endl;

  if (m_deb_func)
  {
    StampedPoseVector poses;
    for (std::map<int, VertexPoseConstPtr>::const_iterator it = m_reference_vxs.begin(); it != m_reference_vxs.end(); ++it)
      poses.push_back(std::make_pair(static_cast<const FrameData *>(it->second->userData())->m_timestamp, it->second->estimate()));
    m_deb_func(poses);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::setFrustum(const Eigen::Matrix3f &inverse_kcam, int width, int height,
                                          float near_plane, float far_plane)
{
  const Eigen::Vector3f &tl = inverse_kcam.col(2);
  Eigen::Vector3f tr = tl;
  tr[0] += width * inverse_kcam(0, 0);
  Eigen::Vector3f bl = tl;
  bl[1] += height * inverse_kcam(1, 1);
  const Eigen::Vector3f br(tr[0], bl[1], 1.f);
  const Eigen::Vector3f npl(0, 0, near_plane);
  const Eigen::Vector3f fpl(0, 0, far_plane);

  m_frustum_mtx.row(0).head<3>() = Eigen::Vector3f::UnitZ();
  m_frustum_mtx(0, 3) = -1 * m_frustum_mtx.row(0).head<3>().dot(fpl); // far
  m_frustum_mtx.row(1).head<3>() = -1 * m_frustum_mtx.row(0).head<3>();
  m_frustum_mtx(1, 3) = m_frustum_mtx.row(0).head<3>().dot(npl); // near
  m_frustum_mtx.row(2).head<3>() = tr.cross(tl).normalized();
  m_frustum_mtx(2, 3) = 0.f; // top
  m_frustum_mtx.row(3).head<3>() = bl.cross(br).normalized();
  m_frustum_mtx(3, 3) = 0.f; // bottom
  m_frustum_mtx.row(4).head<3>() = tl.cross(bl).normalized();
  m_frustum_mtx(4, 3) = 0.f; // left
  m_frustum_mtx.row(5).head<3>() = br.cross(tr).normalized();
  m_frustum_mtx(5, 3) = 0.f; // right
}

/////////////////////////////////////////////////////////////////////////////////////////////////
namespace
{
enum
{
  SLAM_DUNK_FEATURE_GRID_SIZE = 4
};
}

bool slamdunk::FeatureTracker::track(const RGBDFrame &frame)
{
  std::vector<cv::KeyPoint> kpts;
  extractFrameData(frame, m_last_tracked.m_frame_data, &kpts);
  if (m_last_tracked.m_frame_data.m_keypoints.size() < m_min_matches)
  {
    if (m_verbose)
      std::cout << SLAM_DUNK_WARNING_STR("Too few valid visual features...track failure") << std::endl;
    return false;
  }

  // frame-frame matching
  std::vector<FMatch> ff_matches;
  {
    SLAM_DUNK_AUTO_CPU_TIMER("Feature Matching");
    m_matcher->match(m_last_tracked.m_frame_data.m_descriptors, ff_matches);
  }
  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR(ff_matches.size() << " matches found with previous frames") << std::endl;
#ifndef NDEBUG
  if (m_debug)
  {
    std::map<int, std::vector<cv::DMatch>> ref2matches;
    for (unsigned i = 0; i < ff_matches.size(); ++i)
    {
      std::vector<cv::DMatch> &dmatches = ref2matches[ff_matches[i].m_model_idx];
      dmatches.push_back(cv::DMatch(ff_matches[i].m_feat_idx, ff_matches[i].m_query_idx, 0));
    }
    for (std::map<int, std::vector<cv::DMatch>>::const_iterator dmit = ref2matches.begin(); dmit != ref2matches.end(); ++dmit)
    {
      VertexPoseConstPtr frame_vx = m_reference_vxs[dmit->first];
      std::cout << SLAM_DUNK_DEBUG_STR(dmit->second.size() << " matches with reference vx #" << frame_vx->id()) << std::endl;
      const FrameData *fd = static_cast<const FrameData *>(frame_vx->userData());
      cv::Mat outimg;
      cv::drawMatches(fd->m_image, fd->m_kpts2d,
                      m_last_tracked.m_frame_data.m_image, m_last_tracked.m_frame_data.m_kpts2d,
                      dmit->second, outimg);
      cv::imshow("debug", outimg);
      cv::waitKey();
    }
  }
#endif
  if (ff_matches.size() < m_min_matches)
  {
    if (m_verbose)
      std::cout << SLAM_DUNK_WARNING_STR("Too few feature matches...track failure") << std::endl;
    return false;
  }

  // outlier rejection
  std::vector<Eigen::Vector3f> ff_ref_kpts(ff_matches.size());
  std::vector<std::pair<unsigned, unsigned>> ref_query_ids(ff_matches.size());
  std::vector<VertexPoseConstPtr> ref_vxs(ff_matches.size());
  for (unsigned i = 0; i < ff_matches.size(); ++i)
  {
    VertexPoseConstPtr frame_vx = m_reference_vxs[ff_matches[i].m_model_idx];

    ff_ref_kpts[i] = frame_vx->estimate().cast<float>() *
                     (static_cast<const FrameData *>(frame_vx->userData())->m_keypoints[ff_matches[i].m_feat_idx]);
    ref_query_ids[i] = std::pair<unsigned, unsigned>(i, ff_matches[i].m_query_idx);
    ref_vxs[i] = frame_vx;
  }
  {
    SLAM_DUNK_AUTO_CPU_TIMER("Outlier Rejection");
    m_outlier_rejection->findInliers(ff_ref_kpts, m_last_tracked.m_frame_data.m_keypoints, ref_query_ids);
  }
  const std::vector<unsigned> &rqi_inliers = m_outlier_rejection->getRefQueryInliers();
  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR(rqi_inliers.size() << " inliers survived") << std::endl;
#ifndef NDEBUG
  if (m_debug)
  {
    std::map<VertexPoseConstPtr, std::vector<cv::DMatch>> ref2matches;
    for (unsigned i = 0; i < rqi_inliers.size(); ++i)
    {
      const std::pair<unsigned, unsigned> &inlier_pair = ref_query_ids[rqi_inliers[i]];
      std::vector<cv::DMatch> &dmatches = ref2matches[ref_vxs[inlier_pair.first]];
      dmatches.push_back(cv::DMatch(ff_matches[inlier_pair.first].m_feat_idx, inlier_pair.second, 0));
    }
    for (std::map<VertexPoseConstPtr, std::vector<cv::DMatch>>::const_iterator dmit = ref2matches.begin(); dmit != ref2matches.end(); ++dmit)
    {
      const VertexPoseConstPtr frame_vx = dmit->first;
      std::cout << SLAM_DUNK_DEBUG_STR(dmit->second.size() << " matches with reference vx #" << frame_vx->id()) << std::endl;
      const FrameData *fd = static_cast<const FrameData *>(frame_vx->userData());
      cv::Mat outimg;
      cv::drawMatches(fd->m_image, fd->m_kpts2d,
                      m_last_tracked.m_frame_data.m_image, m_last_tracked.m_frame_data.m_kpts2d,
                      dmit->second, outimg);
      cv::imshow("debug", outimg);
      cv::waitKey();
    }
  }
#endif
  if (rqi_inliers.size() < m_min_matches)
  {
    if (m_verbose)
      std::cout << SLAM_DUNK_WARNING_STR("Too few feature matches...track failure") << std::endl;
    return false;
  }

  // get the inliers
  bool inlier_grid[SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE];
  //bool feature_grid[SLAM_DUNK_FEATURE_GRID_SIZE*SLAM_DUNK_FEATURE_GRID_SIZE];
  std::memset(inlier_grid, false, SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE * sizeof(bool));
  //std::memset(feature_grid, false, SLAM_DUNK_FEATURE_GRID_SIZE*SLAM_DUNK_FEATURE_GRID_SIZE*sizeof(bool));

  //int min_x = frame.m_color_image.cols, max_x = 0, min_y = frame.m_color_image.rows, max_y = 0;

  m_last_tracked.m_ff_matches.resize(rqi_inliers.size());
  std::vector<std::pair<unsigned, unsigned>> ref_query_ids_inliers;

  m_last_tracked_keyframes.clear();
  for (unsigned i = 0; i < rqi_inliers.size(); ++i)
  {
    const std::pair<unsigned, unsigned> &inlier_pair = ref_query_ids[rqi_inliers[i]];
    ref_query_ids_inliers.push_back(inlier_pair);

    FrameToFrameMatch &ffm = m_last_tracked.m_ff_matches[i];
    ffm.m_matching_frame_id = ref_vxs[inlier_pair.first]->id();
    ffm.m_matching_frame_feat = ff_matches[inlier_pair.first].m_feat_idx;
    ffm.m_ref_frame_feat = inlier_pair.second;
    ffm.m_score = ff_matches[inlier_pair.first].m_match_score;

    m_last_tracked_keyframes.insert(ffm.m_matching_frame_id);

    const cv::KeyPoint &kp = kpts[ffm.m_ref_frame_feat];
    //if(kp.pt.x < min_x) min_x = kp.pt.x;
    //if(kp.pt.x > max_x) max_x = kp.pt.x;
    //if(kp.pt.y < min_y) min_y = kp.pt.y;
    //if(kp.pt.y > max_y) max_y = kp.pt.y;

    const int ug = (int)(kp.pt.x * (float)SLAM_DUNK_FEATURE_GRID_SIZE / (float)frame.m_color_image.cols);
    const int vg = (int)(kp.pt.y * (float)SLAM_DUNK_FEATURE_GRID_SIZE / (float)frame.m_color_image.rows);
    assert(ug >= 0 && ug < SLAM_DUNK_FEATURE_GRID_SIZE);
    assert(vg >= 0 && vg < SLAM_DUNK_FEATURE_GRID_SIZE);
    inlier_grid[ug + vg * SLAM_DUNK_FEATURE_GRID_SIZE] = true;
  }
  //for(unsigned i = 0; i < m_last_tracked.m_frame_data.m_keypoints.size(); ++i)
  //{
  //const cv::KeyPoint& kp = kpts[i];
  //const int ug = (int)(kp.pt.x * (float)SLAM_DUNK_FEATURE_GRID_SIZE / (float)frame.m_color_image.cols);
  //const int vg = (int)(kp.pt.y * (float)SLAM_DUNK_FEATURE_GRID_SIZE / (float)frame.m_color_image.rows);
  //assert(ug >= 0 && ug < SLAM_DUNK_FEATURE_GRID_SIZE);
  //assert(vg >= 0 && vg < SLAM_DUNK_FEATURE_GRID_SIZE);
  //feature_grid[ug + vg*SLAM_DUNK_FEATURE_GRID_SIZE] = true;
  //}

  // pose estimation
  {
    SLAM_DUNK_AUTO_CPU_TIMER("SVD Pose Estimation");
    slamdunk::estimateTransformationSVD(ff_ref_kpts, m_last_tracked.m_frame_data.m_keypoints, ref_query_ids_inliers, m_last_tracked.m_pose);
  }
  const Eigen::Isometry3d incremental_pose = m_last_tracked.m_pose.inverse() * m_active_win_pose;
  const double cosTheta = (incremental_pose.rotation().trace() - 1.) * 0.5;
  const float angleScore = (float)(cosTheta < 0.707106781 ? 0. : std::pow(2. * cosTheta * cosTheta - 1., 2));
  // overlapping score computation
  m_last_tracked.m_overlapping_score = 0.33333333f * ((float)std::count(inlier_grid, inlier_grid + SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE, true) / (float)(SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE) + angleScore + (float)(1. - std::min(1., incremental_pose.translation().norm() * 2.5 / m_half_active_win_length)));

  //0.5f * ((float)std::count(inlier_grid, inlier_grid+SLAM_DUNK_FEATURE_GRID_SIZE*SLAM_DUNK_FEATURE_GRID_SIZE, true)
  /// (float)std::count(feature_grid, feature_grid+SLAM_DUNK_FEATURE_GRID_SIZE*SLAM_DUNK_FEATURE_GRID_SIZE, true) +
  //(float)(max_x-min_x)*(max_y-min_y)/(float)(frame.m_color_image.cols*frame.m_color_image.rows));

  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR("Pose estimated by SVD (overlapping score: ")
              << ((float)std::count(inlier_grid, inlier_grid + SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE, true) / (float)(SLAM_DUNK_FEATURE_GRID_SIZE * SLAM_DUNK_FEATURE_GRID_SIZE)) / 3. << "+"
              << angleScore / 3.f << "+"
              << (1. - std::min(1., incremental_pose.translation().norm() * 2.5 / m_half_active_win_length)) / 3. << "="
              << m_last_tracked.m_overlapping_score << ")" << std::endl;

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::updateMap()
{
  // should we move the active window?
  if ((((Eigen::Vector2d(m_last_tracked.m_pose.translation().x(), m_last_tracked.m_pose.translation().z()) -
         m_active_win_center)
            .cwiseAbs() -
        m_win_movement_step)
           .array() > 0)
          .any())
  {
    if (m_verbose)
      std::cout << SLAM_DUNK_INFO_STR("Moving active window") << std::endl;
    calcActiveWindow();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::FeatureTracker::extractFrameData(const RGBDFrame &frame, FrameData &frame_data) const
{
  extractFrameData(frame, frame_data, NULL);
}

void slamdunk::FeatureTracker::extractFrameData(const RGBDFrame &frame, FrameData &frame_data, std::vector<cv::KeyPoint> *kpts_ptr) const
{
  frame_data.m_timestamp = frame.m_timestamp;
  // feature extraction
  std::vector<cv::KeyPoint> kpts;
  cv::Mat cvdesc;
  {
    SLAM_DUNK_AUTO_CPU_TIMER("Feature Extraction");
    (*m_feature_extractor)(frame.m_color_image, cv::Mat(), kpts, cvdesc, false);
  }
  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR(cvdesc.rows << " features extracted") << std::endl;
#ifndef NDEBUG
  if (m_debug)
  {
    std::cout << SLAM_DUNK_DEBUG_STR("Extracted Keypoints") << std::endl;
    cv::Mat outimg;
    if (frame.m_color_image.channels() == 3)
      outimg = frame.m_color_image.clone();
    else
      cv::cvtColor(frame.m_color_image, outimg, CV_GRAY2BGR);
    cv::drawKeypoints(outimg, kpts, outimg, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("debug", outimg);
    cv::waitKey();
  }
#endif

  // keep only feats with a valid depth value
  cv::Mat valid_desc(cvdesc.rows, cvdesc.cols, cvdesc.type());
  unsigned feat_with_depth = 0;
  frame_data.m_keypoints.clear();
#ifndef NDEBUG
  frame_data.m_kpts2d.clear();
  frame_data.m_image = frame.m_color_image.clone();
#endif
  for (unsigned i = 0; i < kpts.size() && feat_with_depth < m_max_feats_per_frame; ++i)
  {
    assert(kpts[0].response >= kpts[i].response);
    const float &z = frame.m_depth_image(kpts[i].pt);
    if (z > 0)
    {
      const Eigen::Vector3f pt3d = m_inverse_kcam * Eigen::Vector3f(kpts[i].pt.x * z, kpts[i].pt.y * z, z);
      assert(isfinite(pt3d.x()) && isfinite(pt3d.y()) && isfinite(pt3d.z()) && std::abs(pt3d.z()) > 0.2);
      frame_data.m_keypoints.push_back(pt3d);
      if (kpts_ptr != NULL)
        kpts_ptr->push_back(kpts[i]);
#ifndef NDEBUG
      frame_data.m_kpts2d.push_back(kpts[i]);
#endif
      cvdesc.row(i).copyTo(valid_desc.row(feat_with_depth));

      ++feat_with_depth;
    }
  }
  frame_data.m_descriptors = valid_desc.rowRange(0, feat_with_depth);

  if (m_verbose)
    std::cout << SLAM_DUNK_INFO_STR(frame_data.m_descriptors.rows << " features have a valid depth value") << std::endl;
#ifndef NDEBUG
  if (m_debug)
  {
    std::cout << SLAM_DUNK_DEBUG_STR("Valid Keypoints") << std::endl;
    cv::Mat imgleft, imgright, outimg;
    if (frame.m_color_image.channels() == 3)
    {
      imgleft = frame.m_color_image.clone();
      imgright = frame.m_color_image.clone();
    }
    else
    {
      cv::cvtColor(frame.m_color_image, imgleft, CV_GRAY2BGR);
      cv::cvtColor(frame.m_color_image, imgright, CV_GRAY2BGR);
    }
    cv::drawKeypoints(imgleft, kpts, imgleft, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(imgright, frame_data.m_kpts2d, imgright, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    outimg.create(imgleft.rows, imgleft.cols + imgright.cols, imgleft.type());
    imgleft.copyTo(outimg.colRange(0, imgleft.cols));
    imgright.copyTo(outimg.colRange(imgleft.cols, outimg.cols));

    cv::imshow("debug", outimg);
    cv::waitKey();
  }
#endif
}
