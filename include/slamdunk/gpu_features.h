/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_GPU_FEATURES_H
#define SLAM_DUNK_GPU_FEATURES_H

#include "slamdunk/slamdunk_defines.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/nonfree/gpu.hpp"

namespace slamdunk
{

class SLAM_DUNK_API GPUSurf : public cv::Feature2D
{
public:
  explicit GPUSurf(double hessianThreshold, int nOctaves = 4,
                   int nOctaveLayers = 2, bool extended = false, float keypointsRatio = 0.01f, bool upright = false);

  //! returns the descriptor size in floats (64 or 128)
  inline int descriptorSize() const { return m_surf_gpu.descriptorSize(); }

  //! returns the descriptor type
  inline int descriptorType() const { return CV_32F; }

  //! finds the keypoints using SIFT algorithm
  void operator()(cv::InputArray img, cv::InputArray mask,
                  std::vector<cv::KeyPoint> &keypoints) const;
  //! finds the keypoints and computes descriptors for them using SIFT algorithm.
  //! Optionally it can compute descriptors for the user-provided keypoints
  void operator()(cv::InputArray img, cv::InputArray mask,
                  std::vector<cv::KeyPoint> &keypoints,
                  cv::OutputArray descriptors,
                  bool useProvidedKeypoints = false) const;

  // This function has little sense outside of an OpenCV module
  cv::AlgorithmInfo *info() const { return NULL; };

protected:
  void detectImpl(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, const cv::Mat &mask = cv::Mat()) const;
  void computeImpl(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const;

  mutable cv::gpu::SURF_GPU m_surf_gpu;            // cv::SURF_GPU needs to change its state during feature extraction
  mutable cv::gpu::GpuMat m_image_gpu, m_mask_gpu; // we need to upload the image onto the device before actual computation
};

typedef GPUSurf GPUSurfFeatureDetector;
typedef GPUSurf GPUSurfDescriptorExtractor;
}

#endif // SLAM_DUNK_GPU_FEATURES_H
