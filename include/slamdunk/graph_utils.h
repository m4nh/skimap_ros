
#ifndef SLAM_DUNK_GRAPH_UTILS_H
#define SLAM_DUNK_GRAPH_UTILS_H

#include "slamdunk/slamdunk_defines.h"
#include <Eigen/Geometry>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/sparse_optimizer.h>

namespace slamdunk
{

  typedef std::map<int, Eigen::Isometry3d> PoseIsometryMap;

  /**
   * \brief stop iterating based on the gain which is (oldChi - currentChi) / currentChi.
   *
   * If the gain is larger than zero and below the threshold, then the optimizer is stopped.
   * Typically usage of this action includes adding it as a postIteration action, by calling
   * addPostIterationAction on a sparse optimizer.
   */
  class SLAM_DUNK_API ConvergenceCheckAction : public g2o::HyperGraphAction
  {
    public:
      ConvergenceCheckAction(g2o::SparseOptimizer* opt, double error_th = 1e-8)
        : m_optimizer(opt), m_gain_th(error_th), m_last_chi2(0.), m_stop_flag(false)
      {}

      void reset(g2o::SparseOptimizer* opt = NULL, double error_th = -1.)
      {
        if(opt != NULL)
          m_optimizer = opt;
        if(error_th >= 0)
          m_gain_th = error_th;
        m_last_chi2 = 0.;
        m_stop_flag = false;
      }

#ifdef NDEBUG
      virtual g2o::HyperGraphAction* operator()(const g2o::HyperGraph*, g2o::HyperGraphAction::Parameters* = 0)
#else
      virtual g2o::HyperGraphAction* operator()(const g2o::HyperGraph* graph, g2o::HyperGraphAction::Parameters* = 0)
#endif
      {
        assert(graph == m_optimizer);

        m_optimizer->computeActiveErrors();
        const double current_chi2 = m_optimizer->activeChi2();
        const double gain = m_last_chi2/current_chi2 - 1.0;
        if((gain >= 0 && gain < m_gain_th) || current_chi2 == 0.0)
        {
          // tell the optimizer to stop
          if(m_optimizer->forceStopFlag() != NULL)
            *(m_optimizer->forceStopFlag()) = true;
          else
          {
            m_stop_flag = true;
            m_optimizer->setForceStopFlag(&m_stop_flag);
          }
        }
        m_last_chi2 = current_chi2;
        return this;
      }

    private:
      g2o::SparseOptimizer* m_optimizer;
      double m_gain_th, m_last_chi2;
      bool m_stop_flag;
  };

  class GraphBackend;
  // The root is yellow, the colored orange if the id != root_id,
  //    the highlighted are light blue if they are not colored and the id != root_id
  SLAM_DUNK_API std::string graphToDot(const GraphBackend* gb, int root_id = 0,
                                    const g2o::HyperGraph::VertexSet& colored = g2o::HyperGraph::VertexSet(),
                                    const g2o::HyperGraph::VertexSet& highlighted = g2o::HyperGraph::VertexSet(),
                                    const std::string& graph_name = "mygraph");

}

#endif // SLAM_DUNK_GRAPH_UTILS_H
