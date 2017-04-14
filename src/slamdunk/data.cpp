/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "slamdunk/data.h"
#include <iomanip>

// TODO: read/write Mat type

slamdunk::RGBDFrame slamdunk::cloneRGBDFrame(const slamdunk::RGBDFrame &frame)
{
  RGBDFrame f;
  f.m_timestamp = frame.m_timestamp;
  f.m_color_image = frame.m_color_image.clone();
  f.m_depth_image = frame.m_depth_image.clone();
  return f;
}

bool slamdunk::FrameData::read(std::istream &is)
{
  is >> m_timestamp;
  int rows, cols;
  is >> rows >> cols;
  m_descriptors.create(rows, cols, CV_32F);
  m_keypoints.resize(rows);
  for (int i = 0; i < rows; ++i)
  {
    float *dptr = m_descriptors.ptr<float>(i);
    for (int j = 0; j < cols; ++j)
      is >> dptr[j];
  }
  for (int i = 0; i < rows; ++i)
  {
    is >> m_keypoints[i].x() >> m_keypoints[i].y() >> m_keypoints[i].z();
    m_keypoints[i][3] = 1.0;
  }
  return true;
}

bool slamdunk::FrameData::write(std::ostream &os) const
{
  os << m_timestamp << " ";
  assert((int)m_keypoints.size() == m_descriptors.rows);
  // descriptors
  os << m_descriptors.rows << " " << m_descriptors.cols;
  for (int i = 0; i < m_descriptors.rows; ++i)
  {
    const float *dptr = m_descriptors.ptr<float>(i);
    for (int j = 0; j < m_descriptors.cols; ++j)
      os << " " << std::fixed << std::setprecision(15) << dptr[j];
  }
  // keypoints
  for (unsigned i = 0; i < m_keypoints.size(); ++i)
    os << " " << std::fixed << std::setprecision(15) << m_keypoints[i].x() << " " << m_keypoints[i].y() << " " << m_keypoints[i].z();
  return os.good();
}

cv::Mat_<float> slamdunk::depthBilateralFilter(const cv::Mat_<float> &input, double sigmaDepth, double sigmaSpace)
{
  cv::Mat_<float> output(input.rows, input.cols);

  double one_over_sigmaD22 = 1. / (2. * sigmaDepth * sigmaDepth);
  double one_over_sigmaS22 = 1. / (2. * sigmaSpace * sigmaSpace);
  double one_over_sigmaD2Pi = 1. / (sigmaDepth * std::sqrt(2. * M_PI));
  double one_over_sigmaS2Pi = 1. / (sigmaSpace * std::sqrt(2. * M_PI));

  int radius = (int)std::ceil(2. * sigmaSpace);
  // spatial weights
  Eigen::MatrixXd ws(2 * radius + 1, 2 * radius + 1);
  for (int y = -radius; y < radius + 1; ++y)
    for (int x = -radius; x < radius + 1; ++x)
      ws(y + radius, x + radius) = std::exp(-1. * (y * y + x * x) * one_over_sigmaS22) * one_over_sigmaS2Pi;

  for (int v = 0; v < input.rows; ++v)
    for (int u = 0; u < input.cols; ++u)
      if (input(v, u) > 0)
      {
        double value = 0., norm_factor = 0.;
        for (int y = v - radius; y < v + radius + 1; ++y)
          for (int x = u - radius; x < u + radius + 1; ++x)
          {
            if (x >= 0 && x < input.cols && y >= 0 && y < input.rows && input(y, x) > 0)
            {
              double depth_diff = input(y, x) - input(v, u);
              double gauss_product = ws(y - v + radius, x - u + radius) * std::exp(-1. * depth_diff * depth_diff * one_over_sigmaD22) * one_over_sigmaD2Pi;
              value += gauss_product * input(y, x);
              norm_factor += gauss_product;
            }
          }
        output(v, u) = (float)(value / norm_factor);
      }
      else
        output(v, u) = 0.f;

  return output;
}
