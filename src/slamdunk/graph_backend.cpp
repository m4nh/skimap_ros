/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "slamdunk/graph_backend.h"
#include "slamdunk/pretty_printer.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

slamdunk::GraphBackend::GraphBackend()
    : m_optimizer(), m_action(&m_optimizer, 1e-10), m_edge_weights(0.)
{
#if defined(_MSC_VER) && EIGEN_WORLD_VERSION >= 3 && EIGEN_MAJOR_VERSION >= 2
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, Eigen::Dynamic>> BlockSolver_6_0;
#else
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 0>> BlockSolver_6_0;
#endif
  // CholMod solver
  //BlockSolver_6_0::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<BlockSolver_6_0::PoseMatrixType>();
  // PCG solver
  BlockSolver_6_0::LinearSolverType *linearSolver = new g2o::LinearSolverPCG<BlockSolver_6_0::PoseMatrixType>();

  BlockSolver_6_0 *solver_ptr = new BlockSolver_6_0(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *opt_alg = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  // set the algorithm and the termination action
  m_optimizer.setAlgorithm(opt_alg);
  m_optimizer.addPostIterationAction(&m_action);

  g2o::ParameterSE3Offset *offset = new g2o::ParameterSE3Offset();
  offset->setId(0);
  m_optimizer.addParameter(offset);
}

void slamdunk::GraphBackend::setFixedView(int view_id, bool fixed)
{
  g2o::SparseOptimizer::Vertex *v = getPose(view_id, false);
  if (v != NULL)
    v->setFixed(fixed);
}

g2o::VertexSE3 *slamdunk::GraphBackend::setVertexEstimate(int id, const Eigen::Isometry3d &estimate, FrameData *data)
{
  // get/create the vertex
  g2o::VertexSE3 *vx = static_cast<g2o::VertexSE3 *>(getPose(id, true));
  vx->setEstimate(estimate);
  vx->setUserData(data);
  return vx;
}

void slamdunk::GraphBackend::setVertexData(int id, FrameData *data)
{
  g2o::SparseOptimizer::Vertex *vx = getPose(id, false);
  if (vx != NULL)
    vx->setUserData(data);
}

slamdunk::FrameData *slamdunk::GraphBackend::getVertexData(int id)
{
  g2o::SparseOptimizer::Vertex *vx = getPose(id, false);
  if (vx != NULL)
    return static_cast<FrameData *>(vx->userData());
  return NULL;
}

const slamdunk::FrameData *slamdunk::GraphBackend::getVertexData(int id) const
{
  const g2o::SparseOptimizer::Vertex *vx = getPose(id);
  if (vx != NULL)
    return static_cast<const FrameData *>(vx->userData());
  return NULL;
}

bool slamdunk::GraphBackend::getVertexEstimate(int id, Eigen::Isometry3d &tr) const
{
  const g2o::VertexSE3 *vx = static_cast<const g2o::VertexSE3 *>(getPose(id));
  if (!vx)
    return false;
  tr = vx->estimate();
  return true;
}

slamdunk::FrameData *slamdunk::GraphBackend::getVertexEstimateAndData(int id, Eigen::Isometry3d &tr)
{
  g2o::VertexSE3 *vx = static_cast<g2o::VertexSE3 *>(getPose(id, false));
  if (!vx)
    return NULL;
  tr = vx->estimate();
  return static_cast<FrameData *>(vx->userData());
}

const slamdunk::FrameData *slamdunk::GraphBackend::getVertexEstimateAndData(int id, Eigen::Isometry3d &tr) const
{
  const g2o::VertexSE3 *vx = static_cast<const g2o::VertexSE3 *>(getPose(id));
  if (!vx)
    return NULL;
  tr = vx->estimate();
  return static_cast<const FrameData *>(vx->userData());
}

bool slamdunk::GraphBackend::getRelativeTransformation(int id_from, int id_to, Eigen::Isometry3d &tr) const
{
  const g2o::VertexSE3 *v_from = static_cast<const g2o::VertexSE3 *>(getPose(id_from));
  const g2o::VertexSE3 *v_to = static_cast<const g2o::VertexSE3 *>(getPose(id_to));
  if (!v_from || !v_to)
    return false;
  tr = v_to->estimate().inverse() * v_from->estimate();
  return true;
}

void slamdunk::GraphBackend::updatePoses(const PoseIsometryMap &poses)
{
  if (m_optimizer.vertices().empty())
    return;
  Eigen::Isometry3d pivot_to_world;
  PoseIsometryMap::const_iterator pivot_it = poses.end();
  for (PoseIsometryMap::const_iterator it = poses.begin(); it != poses.end(); ++it)
  {
    const g2o::SparseOptimizer::Vertex *vx = getPose(it->first);
    if (vx != NULL)
    {
      pivot_to_world = static_cast<const g2o::VertexSE3 *>(vx)->estimate();
      pivot_it = it;
      break;
    }
  }
  if (pivot_it == poses.end())
    return;
  Eigen::Isometry3d frame_transformation = pivot_to_world * pivot_it->second.inverse();

  if (poses.size() > m_optimizer.vertices().size())
  {
    for (g2o::HyperGraph::VertexIDMap::iterator it = m_optimizer.vertices().begin(); it != m_optimizer.vertices().end(); ++it)
    {
      const PoseIsometryMap::const_iterator uppose_it = poses.find(it->second->id());
      if (uppose_it != poses.end())
        static_cast<g2o::VertexSE3 *>(it->second)->setEstimate(frame_transformation * uppose_it->second);
    }
  }
  else
  {
    for (PoseIsometryMap::const_iterator uppose_it = poses.begin(); uppose_it != poses.end(); ++uppose_it)
    {
      g2o::SparseOptimizer::Vertex *vx = getPose(uppose_it->first, false);
      if (vx)
        static_cast<g2o::VertexSE3 *>(vx)->setEstimate(frame_transformation * uppose_it->second);
    }
  }
}

void slamdunk::GraphBackend::setPoses(const PoseIsometryMap &poses)
{
  if (poses.size() > m_optimizer.vertices().size())
  {
    for (g2o::HyperGraph::VertexIDMap::iterator it = m_optimizer.vertices().begin(); it != m_optimizer.vertices().end(); ++it)
    {
      const PoseIsometryMap::const_iterator uppose_it = poses.find(it->second->id());
      if (uppose_it != poses.end())
        static_cast<g2o::VertexSE3 *>(it->second)->setEstimate(uppose_it->second);
    }
  }
  else
  {
    for (PoseIsometryMap::const_iterator uppose_it = poses.begin(); uppose_it != poses.end(); ++uppose_it)
    {
      g2o::SparseOptimizer::Vertex *vx = getPose(uppose_it->first, false);
      if (vx)
        static_cast<g2o::VertexSE3 *>(vx)->setEstimate(uppose_it->second);
    }
  }
}

void slamdunk::GraphBackend::removePose(int id)
{
  g2o::SparseOptimizer::Vertex *vx = getPose(id, false);
  if (!vx)
    return;
  for (g2o::HyperGraph::EdgeSet::const_iterator it = vx->edges().begin(); it != vx->edges().end(); ++it)
    m_edge_weights -= static_cast<const EdgeSE3XYZPair *>(*it)->weight();
  m_optimizer.removeVertex(vx);
}

void slamdunk::GraphBackend::addFrameMatchesXYZ(int referenceId, const std::vector<FrameToFrameMatch> &matches,
                                                std::set<int> *matchingFrames)
{
  // get/create the vertex
  g2o::SparseOptimizer::Vertex *ref_vx = getPose(referenceId, true);
  const FrameData *ref_data = static_cast<const FrameData *>(ref_vx->userData());
  for (size_t i = 0; i < matches.size(); ++i)
  {
    // get/create the vertex
    g2o::SparseOptimizer::Vertex *matching_vx = getPose(matches[i].m_matching_frame_id, false);
    if (!matching_vx)
      return;
    const FrameData *matching_data = static_cast<const FrameData *>(matching_vx->userData());
    if (matchingFrames != NULL)
      matchingFrames->insert(matches[i].m_matching_frame_id);

    EdgeSE3XYZPair *edge = new EdgeSE3XYZPair();
    edge->setParameterId(0, 0);
    edge->setParameterId(1, 0);
    edge->vertices()[0] = ref_vx;
    edge->vertices()[1] = matching_vx;
    XYZPairMeasurement meas;
    meas.col(0) = ref_data->m_keypoints[matches[i].m_ref_frame_feat].cast<double>();
    meas.col(1) = matching_data->m_keypoints[matches[i].m_matching_frame_feat].cast<double>();
    edge->setMeasurement(meas);
    if (matches[i].m_score > 0)
      edge->setWeight(matches[i].m_score);
    edge->setPointPointMetric();
    m_optimizer.addEdge(edge);

    m_edge_weights += edge->weight();
  }
}

g2o::SparseOptimizer::Edge *slamdunk::GraphBackend::addFrameMatchXYZ(int frameId1, const Eigen::Vector3d &measurement1,
                                                                     int frameId2, const Eigen::Vector3d &measurement2,
                                                                     double score)
{
  // get/create the vertices
  g2o::SparseOptimizer::Vertex *v1 = getPose(frameId1, true);
  g2o::SparseOptimizer::Vertex *v2 = getPose(frameId2, true);

  EdgeSE3XYZPair *edge = new EdgeSE3XYZPair();
  edge->setParameterId(0, 0);
  edge->setParameterId(1, 0);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  XYZPairMeasurement meas;
  meas.col(0) = measurement1;
  meas.col(1) = measurement2;
  edge->setMeasurement(meas);
  if (score > 0)
    edge->setWeight(score);
  edge->setPointPointMetric();
  m_optimizer.addEdge(edge);

  m_edge_weights += edge->weight();
  return edge;
}

g2o::SparseOptimizer::Edge *slamdunk::GraphBackend::addFrameMatchXYZ(int frameId1, const Eigen::Vector3d &measurement1,
                                                                     const Eigen::Vector3d &normal1,
                                                                     int frameId2, const Eigen::Vector3d &measurement2,
                                                                     double score)
{
  // get/create the vertices
  g2o::SparseOptimizer::Vertex *v1 = getPose(frameId1, true);
  g2o::SparseOptimizer::Vertex *v2 = getPose(frameId2, true);

  EdgeSE3XYZPair *edge = new EdgeSE3XYZPair();
  edge->setParameterId(0, 0);
  edge->setParameterId(1, 0);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  XYZPairMeasurement meas;
  meas.col(0) = measurement1;
  meas.col(1) = measurement2;
  edge->setMeasurement(meas);
  if (score > 0)
    edge->setWeight(score);
  edge->setPointPlaneMetric(normal1, 0.001);
  m_optimizer.addEdge(edge);

  m_edge_weights += edge->weight();
  return edge;
}

g2o::SparseOptimizer::Edge *slamdunk::GraphBackend::addFrameMatchXYZ(int frameId1, const Eigen::Vector3d &measurement1,
                                                                     const Eigen::Vector3d &normal1,
                                                                     int frameId2, const Eigen::Vector3d &measurement2,
                                                                     const Eigen::Vector3d &normal2,
                                                                     double score)
{
  // get/create the vertices
  g2o::SparseOptimizer::Vertex *v1 = getPose(frameId1, true);
  g2o::SparseOptimizer::Vertex *v2 = getPose(frameId2, true);

  EdgeSE3XYZPair *edge = new EdgeSE3XYZPair();
  edge->setParameterId(0, 0);
  edge->setParameterId(1, 0);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  XYZPairMeasurement meas;
  meas.col(0) = measurement1;
  meas.col(1) = measurement2;
  edge->setMeasurement(meas);
  if (score > 0)
    edge->setWeight(score);
  edge->setPlanePlaneMetric(normal1, normal2, 0.001);
  m_optimizer.addEdge(edge);

  m_edge_weights += edge->weight();
  return edge;
}

int slamdunk::GraphBackend::optimize(int max_no_of_iterations)
{
  m_action.reset();
  m_optimizer.initializeOptimization(-1);
  return m_optimizer.optimize(max_no_of_iterations);
}

int slamdunk::GraphBackend::relativeOptimization(int frameId, unsigned noOfRings, int max_no_of_iterations,
                                                 g2o::HyperGraph::VertexSet *vSetPtr, g2o::HyperGraph::VertexSet *fSetPtr)
{
  if (getNoOfVertices() <= 1)
    return 0;
  g2o::SparseOptimizer::Vertex *vx = getPose(frameId, false);
  if (!vx)
    return 0;
  // the active set
  g2o::HyperGraph::VertexSet local_vset, local_fixed_set;
  g2o::HyperGraph::VertexSet *vset = (vSetPtr == NULL ? &local_vset : vSetPtr);
  g2o::HyperGraph::VertexSet *fixed_set = (fSetPtr == NULL ? &local_fixed_set : fSetPtr);
  if (vset->empty())
    createRBASubset(vx, noOfRings, *vset, *fixed_set);
  else
  {
    for (g2o::HyperGraph::VertexSet::iterator it = fixed_set->begin(); it != fixed_set->end(); ++it)
      static_cast<g2o::SparseOptimizer::Vertex *>(*it)->setFixed(true);
  }

  if (m_optimizer.verbose())
    std::cout << SLAM_DUNK_INFO_STR("Inner window: " << (vset->size() - fixed_set->size()))
              << " Outer window: " << fixed_set->size() << std::endl;

  if (fixed_set->empty()) // we need a fixed pose
  {
    fixed_set->insert(vx);
    vx->setFixed(true);
  }

  m_action.reset();
  m_optimizer.initializeOptimization(*vset, -1);
  const int noOfIt = m_optimizer.optimize(max_no_of_iterations);

  for (g2o::HyperGraph::VertexSet::iterator it = fixed_set->begin(); it != fixed_set->end(); ++it)
    static_cast<g2o::SparseOptimizer::Vertex *>(*it)->setFixed(false);

  return noOfIt;
}

void slamdunk::GraphBackend::createRBASubset(g2o::HyperGraph::Vertex *origin, unsigned ring,
                                             g2o::HyperGraph::VertexSet &vset, g2o::HyperGraph::VertexSet &fixedset)
{
  vset.insert(origin);
  g2o::HyperGraph::VertexSet local_sets[2];
  local_sets[0] = vset;
  g2o::HyperGraph::VertexSet *last_ring = &local_sets[0];
  g2o::HyperGraph::VertexSet *next_ring = &local_sets[1];

  for (unsigned cr = 0; cr <= ring; ++cr)
  {
    for (g2o::HyperGraph::VertexSet::const_iterator vit = last_ring->begin(); vit != last_ring->end(); ++vit)
    {
      // collect next ring
      for (g2o::HyperGraph::EdgeSet::iterator eit = (*vit)->edges().begin(); eit != (*vit)->edges().end(); ++eit)
      {
        for (unsigned i = 0; i < (*eit)->vertices().size(); ++i)
        {
          g2o::HyperGraph::Vertex *vi = (*eit)->vertex(i);
          if (!vset.count(vi))
          {
            if (cr == ring)
            {
              vset.insert(vi);
              fixedset.insert(vi);
              static_cast<g2o::SparseOptimizer::Vertex *>(vi)->setFixed(true);
            }
            else
              next_ring->insert(vi);
          }
        } // for every vx attached to the periphery
      }   // for every peripheral edge
    }     // for every peripheral vx
    if (next_ring->empty())
      break;

    vset.insert(next_ring->begin(), next_ring->end());
    std::swap(last_ring, next_ring);
    next_ring->clear();
  } // for every ring
}

void slamdunk::GraphBackend::getOptimizedPoses(PoseIsometryMap &poses) const
{
  for (g2o::HyperGraph::VertexIDMap::const_iterator it = m_optimizer.vertices().begin(); it != m_optimizer.vertices().end(); ++it)
    poses[it->first] = static_cast<const g2o::VertexSE3 *>(it->second)->estimate();
}

g2o::SparseOptimizer::Vertex *slamdunk::GraphBackend::getPose(int pose_id, bool create)
{
  g2o::SparseOptimizer::Vertex *v = m_optimizer.vertex(pose_id);
  if (v == NULL && create)
  {
    g2o::VertexSE3 *vse3 = new g2o::VertexSE3();
    vse3->setId(pose_id);
    vse3->setEstimate(Eigen::Isometry3d::Identity());
    m_optimizer.addVertex(vse3);
    v = vse3;
  }
  return v;
}

const g2o::SparseOptimizer::Vertex *slamdunk::GraphBackend::getPose(int pose_id) const
{
  return m_optimizer.vertex(pose_id);
}
