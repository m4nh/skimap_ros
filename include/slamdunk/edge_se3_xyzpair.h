/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_EDGE_SE3_XYZPAIR_H
#define SLAM_DUNK_EDGE_SE3_XYZPAIR_H

#include "slamdunk/slamdunk_defines.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

namespace slamdunk
{

/** col-0 = point-0; col-1 = point-1 */
typedef Eigen::Matrix<double, 3, 2> XYZPairMeasurement;

class SLAM_DUNK_API EdgeSE3XYZPair : public g2o::BaseBinaryEdge<3, XYZPairMeasurement, g2o::VertexSE3, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3XYZPair();

  inline void setWeight(double w) { weight_ = w; }

  inline void setPointPointMetric()
  {
    plane_plane_metric_ = false;
    _information.setIdentity();
  }

  inline void setPointPlaneMetric(const Eigen::Vector3d &normal0, double e = 1e-3)
  {
    plane_plane_metric_ = false;
    Eigen::Matrix3d info;
    info << e, 0, 0,
        0, e, 0,
        0, 0, 1;
    Eigen::Matrix3d rot0 = computeRotation(normal0);
    _information = rot0.transpose() * info * rot0;
  }

  inline void setPlanePlaneMetric(const Eigen::Vector3d &normal0, const Eigen::Vector3d &normal1, double e = 1e-3)
  {
    plane_plane_metric_ = true;
    Eigen::Matrix3d info;
    info << 1, 0, 0,
        0, 1, 0,
        0, 0, e;

    Eigen::Matrix3d rot = computeRotation(normal0);
    cov0_ = rot.transpose() * info * rot;
    rot = computeRotation(normal1);
    cov1_ = rot.transpose() * info * rot;
  }

  double weight() const { return weight_; }
  const Eigen::Matrix3d &cov0() const { return cov0_; }
  const Eigen::Matrix3d &cov1() const { return cov1_; }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  virtual void computeError();

  virtual bool setMeasurementData(const double *d)
  {
    _measurement = Eigen::Map<const XYZPairMeasurement>(d);
    return true;
  }

  virtual bool getMeasurementData(double *d) const
  {
    Eigen::Map<XYZPairMeasurement> v(d);
    v = _measurement;
    return true;
  }

  virtual void linearizeOplus();

  virtual int measurementDimension() const { return 6; }

private:
  inline Eigen::Matrix3d computeRotation(const Eigen::Vector3d &normal) const
  {
    Eigen::Matrix3d rot;
    rot.row(2) = normal;
    rot.row(1) = (Eigen::Vector3d::UnitY() - normal(1) * normal).normalized();
    rot.row(0) = normal.cross(rot.row(1));
    return rot;
  }

  virtual bool resolveCaches();
  g2o::ParameterSE3Offset *offset0_, *offset1_;
  g2o::CacheSE3Offset *cache0_, *cache1_;

  double weight_;
  bool plane_plane_metric_;
  Eigen::Matrix3d cov0_, cov1_;

  Eigen::Matrix3d jac_buffer_;
};
}

#endif // SLAM_DUNK_EDGE_SE3_XYZPAIR_H
