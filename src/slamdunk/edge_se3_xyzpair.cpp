
#include "slamdunk/edge_se3_xyzpair.h"

slamdunk::EdgeSE3XYZPair::EdgeSE3XYZPair()
  : offset0_(NULL), offset1_(NULL), cache0_(NULL), cache1_(NULL), weight_(1)
{
  jac_buffer_.setZero();
  resizeParameters(2);
  installParameter(offset0_, 0);
  installParameter(offset1_, 1);
}

bool slamdunk::EdgeSE3XYZPair::resolveCaches()
{
  assert(offset0_ && offset1_);

  g2o::ParameterVector pv(2);
  pv[0] = offset0_;
  resolveCache(cache0_, (g2o::OptimizableGraph::Vertex*)_vertices[0], "CACHE_SE3_OFFSET", pv);
  pv[1] = offset1_;
  resolveCache(cache1_, (g2o::OptimizableGraph::Vertex*)_vertices[1], "CACHE_SE3_OFFSET", pv);
  return (cache0_ && cache1_);
}

bool slamdunk::EdgeSE3XYZPair::read(std::istream& is)
{
  int pid0, pid1;
  is >> pid0 >> pid1;
  if(!setParameterId(0, pid0))
    return false;
  if(!setParameterId(1, pid1))
    return false;
  if(!is.good())
    return false;

  is >> _measurement(0,0) >> _measurement(1,0) >> _measurement(2,0)
     >> _measurement(0,1) >> _measurement(1,1) >> _measurement(2,1);
  if(!is.good())
    return false;

  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      is >> _information(i,j);
  if(!is.good())
    return false;

  int plpl;
  is >> weight_ >> plpl;
  plane_plane_metric_ = (plpl == 1 ? true : false);
  if(!is.good())
    return false;

  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      is >> cov0_(i,j);
  if(!is.good())
    return false;

  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      is >> cov1_(i,j);
  return true;
}

bool slamdunk::EdgeSE3XYZPair::write(std::ostream& os) const
{
  os << offset0_->id() << " " << offset1_->id() << " ";
  os << _measurement(0,0) << " " << _measurement(1,0) << " " << _measurement(2,0) << " "
     << _measurement(0,1) << " " << _measurement(1,1) << " " << _measurement(2,1) << " ";
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      os << _information(i,j) << " ";
  os << weight_ << " " << (plane_plane_metric_ ? 1 : 0) << " ";
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      os << cov0_(i,j) << " ";
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      os << cov1_(i,j) << " ";

  return os.good();
}

void slamdunk::EdgeSE3XYZPair::computeError()
{
  if(plane_plane_metric_)
  {
    const Eigen::Isometry3d n2n = cache0_->w2n() * cache1_->n2w();
    _error = weight_ * ((n2n * _measurement.col(1)) - _measurement.col(0));
    _information = (cov0_ + n2n.rotation() * cov1_ * n2n.rotation().transpose()).inverse();
  } else
    _error = weight_ * ((cache0_->w2n() * (cache1_->n2w() * _measurement.col(1))) - _measurement.col(0));
}

// jacobian defined as:
//    f(T0,T1) =  dR0.inv() * T0.inv() * (T1 * dR1 * p1 + dt1) - dt0
//    df/dx0 = [-I, d[dR0.inv()]/dq0 * T01 * p1]
//    df/dx1 = [R0, T01 * d[dR1]/dq1 * p1]
void slamdunk::EdgeSE3XYZPair::linearizeOplus()
{
  const g2o::VertexSE3* vp0 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
  const g2o::VertexSE3* vp1 = static_cast<const g2o::VertexSE3*>(_vertices[1]);

  if (!vp0->fixed())
  {
    // TODO: is it correct??
    //    f(T0,T1) = Off0.inv() * (dR0.inv() * T0.inv() * (T1 * (dR1 * Off1 * p1 + dt1)) - dt0)
    //    df/dx0 = [-Off0.inv().rot(), Off0.inv().rot() * d[dR0.inv()]/dq0 * T01 * Off1 * p1]
    const Eigen::Vector3d p1t = cache0_->w2l() *  (cache1_->n2w() * _measurement.col(1));
    jac_buffer_(1,0) = 2.0*p1t[2];
    jac_buffer_(2,0) = -2.0*p1t[1];
    jac_buffer_(0,1) = -2.0*p1t[2];
    jac_buffer_(2,1) = 2.0*p1t[0];
    jac_buffer_(0,2) = 2.0*p1t[1];
    jac_buffer_(1,2) = -2.0*p1t[0];

    _jacobianOplusXi.block<3,3>(0,0) = -1. * offset0_->inverseOffset().rotation();
    _jacobianOplusXi.block<3,3>(0,3) = offset0_->inverseOffset().rotation() * jac_buffer_;
//    _jacobianOplusXi.block<3,1>(0,3) = offset0_->inverseOffset().rotation() * Eigen::Vector3d(0, 2.0*p1t[2], -2.0*p1t[1]); //dRidx * p1t;
//    _jacobianOplusXi.block<3,1>(0,4) = offset0_->inverseOffset().rotation() * Eigen::Vector3d(-2.0*p1t[2], 0, 2.0*p1t[0]); //dRidy * p1t;
//    _jacobianOplusXi.block<3,1>(0,5) = offset0_->inverseOffset().rotation() * Eigen::Vector3d(2.0*p1t[1], -2.0*p1t[0], 0); //dRidz * p1t;
  }

  if (!vp1->fixed())
  {
    // TODO: is it correct??
    //    f(T0,T1) = Off0.inv() * (dR0.inv() * T0.inv() * (T1 * (dR1 * Off1 * p1 + dt1)) - dt0)
    //    df/dx1 = [Off0.inv().rot() * T0.inv().rot() * T1.rot(), Off0.inv().rot() * T0.inv().rot() * T1.rot() * d[dR1]/dq1 * Off1 * p1]
    const Eigen::Vector3d p1o = offset1_->offset() * _measurement.col(1);
    jac_buffer_(1,0) = -2.0*p1o[2];
    jac_buffer_(2,0) = 2.0*p1o[1];
    jac_buffer_(0,1) = 2.0*p1o[2];
    jac_buffer_(2,1) = -2.0*p1o[0];
    jac_buffer_(0,2) = -2.0*p1o[1];
    jac_buffer_(1,2) = 2.0*p1o[0];

    _jacobianOplusXj.block<3,3>(0,0) = cache0_->w2n().rotation() * vp1->estimate().rotation();
    _jacobianOplusXj.block<3,3>(0,3) = _jacobianOplusXj.block<3,3>(0,0) * jac_buffer_;

//    const Eigen::Matrix3d R0T = vp0->estimate().rotation().transpose() * vp1->estimate().rotation();
//    _jacobianOplusXj.block<3,3>(0,0) = R0T;
//    _jacobianOplusXj.block<3,1>(0,3) = R0T * Eigen::Vector3d(0, -2.0*_measurement(2,1), 2.0*_measurement(1,1)); //R0T * (dRidx.transpose() * _measurement.col(1));
//    _jacobianOplusXj.block<3,1>(0,4) = R0T * Eigen::Vector3d(2.0*_measurement(2,1), 0, -2.0*_measurement(0,1)); //R0T * (dRidy.transpose() * _measurement.col(1));
//    _jacobianOplusXj.block<3,1>(0,5) = R0T * Eigen::Vector3d(-2.0*_measurement(1,1), 2.0*_measurement(0,1), 0); //R0T * (dRidz.transpose() * _measurement.col(1));
  }
}
