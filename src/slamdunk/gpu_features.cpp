
#include "slamdunk/gpu_features.h"
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////////
slamdunk::GPUSurf::GPUSurf(double hessianThreshold, int nOctaves, int nOctaveLayers, bool extended, float keypointsRatio, bool upright)
  : m_surf_gpu(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio, upright)
{}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::GPUSurf::operator()(cv::InputArray _image, cv::InputArray _mask,
                      cv::vector<cv::KeyPoint>& keypoints) const
{
  (*this)(_image, _mask, keypoints, cv::noArray());
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::GPUSurf::operator()(cv::InputArray _image, cv::InputArray _mask,
                                std::vector<cv::KeyPoint>& keypoints,
                                cv::OutputArray _descriptors,
                                bool useProvidedKeypoints) const
{
  cv::Mat image = _image.getMat(), mask = _mask.getMat();

  if( image.empty() || image.depth() != CV_8U )
    CV_Error( CV_StsBadArg, "image is empty or has incorrect depth (!=CV_8U)" );

  if( !mask.empty() && mask.type() != CV_8UC1 )
    CV_Error( CV_StsBadArg, "mask has incorrect type (!=CV_8UC1)" );

  if( image.channels() == 3 )
    cv::cvtColor(image, image, CV_BGR2GRAY);
  else if( image.channels() == 4 )
    cv::cvtColor(image, image, CV_BGRA2GRAY);
  else
    CV_Error( CV_StsBadArg, "image has incorrect no of channels (!=1|3|4)" );

  m_image_gpu.upload(image);
  if(!mask.empty())
    m_mask_gpu.upload(mask);
  else if(!m_mask_gpu.empty())
    m_mask_gpu = cv::gpu::GpuMat();
  cv::gpu::GpuMat descriptors_gpu;
  m_surf_gpu(m_image_gpu, m_mask_gpu, keypoints, descriptors_gpu, useProvidedKeypoints);

  cv::Mat& descriptors = _descriptors.getMatRef();
  descriptors_gpu.download(descriptors);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::GPUSurf::detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  (*this)(image, mask, keypoints, cv::noArray());
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void slamdunk::GPUSurf::computeImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
  (*this)(image, cv::Mat(), keypoints, descriptors, true);
}
