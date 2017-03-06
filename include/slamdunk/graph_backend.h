
#ifndef SLAM_DUNK_GRAPH_BACKEND_H
#define SLAM_DUNK_GRAPH_BACKEND_H

#include "slamdunk/slamdunk_defines.h"
#include "slamdunk/graph_utils.h"
#include "slamdunk/data.h"
#include "slamdunk/edge_se3_xyzpair.h"

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace slamdunk
{

  class SLAM_DUNK_API GraphBackend : private boost::noncopyable
  {
    public:
      typedef boost::shared_ptr<GraphBackend> Ptr;
      typedef boost::shared_ptr<const GraphBackend> ConstPtr;

      GraphBackend();
      ~GraphBackend() {}

      inline void setVerbose(bool verbose) { m_optimizer.setVerbose(verbose); }

      /// The vertex must exists
      void setFixedView(int view_id, bool fixed = true);
      /** Set the estimate (and optionally data) for the vertex with index id
        * NB: the vertex is created if it is not already in the graph
        */
      g2o::VertexSE3* setVertexEstimate(int id, const Eigen::Isometry3d& estimate, FrameData* data = NULL);
      /** Set the aux data for the vertex with index id (PREVIOUSLY SET DATA ARE NOT DELETED)
        * NB: if the vertex does not exist, nothing is done
        */
      void setVertexData(int id, FrameData* data);
      /** Get the aux data for the vertex with index id */
      FrameData* getVertexData(int id);
      const FrameData* getVertexData(int id) const;
      /** Get the estimate for the vertex with index id
        * \return false if the vertex does not exists
        */
      bool getVertexEstimate(int id, Eigen::Isometry3d& tr) const;

      FrameData* getVertexEstimateAndData(int id, Eigen::Isometry3d& tr);
      const FrameData* getVertexEstimateAndData(int id, Eigen::Isometry3d& tr) const;
      /** Get the estimated transformation from vertex id_from to vertex id_to
        * \return false if at least one vertex does not exists
        */
      bool getRelativeTransformation(int id_from, int id_to, Eigen::Isometry3d& tr) const;

      /** Update the relative estimates using the given values */
      void updatePoses(const PoseIsometryMap& poses);
      /** Update the absolute estimates using the given values */
      void setPoses(const PoseIsometryMap& poses);
      /** Remove pose with the given index */
      void removePose(int id);

      /** Add EdgeSE3XYZPair edges (point-point metric) */
      void addFrameMatchesXYZ(int referenceId, const std::vector<FrameToFrameMatch>& matches,
                              std::set<int>* matchingFrames = NULL);
      /** Add an EdgeSE3XYZPair (point-point metric) */
      g2o::SparseOptimizer::Edge* addFrameMatchXYZ( int frameId1, const Eigen::Vector3d& measurement1,
                                                    int frameId2, const Eigen::Vector3d& measurement2,
                                                    double score = 1.0);
      /** Add an EdgeSE3XYZPair (point-plane metric) */
      g2o::SparseOptimizer::Edge* addFrameMatchXYZ( int frameId1, const Eigen::Vector3d& measurement1,
                                                    const Eigen::Vector3d& normal1,
                                                    int frameId2, const Eigen::Vector3d& measurement2,
                                                    double score = 1.0);
      /** Add an EdgeSE3XYZPair (plane-plane metric) */
      g2o::SparseOptimizer::Edge* addFrameMatchXYZ( int frameId1, const Eigen::Vector3d& measurement1,
                                                    const Eigen::Vector3d& normal1,
                                                    int frameId2, const Eigen::Vector3d& measurement2,
                                                    const Eigen::Vector3d& normal2,
                                                    double score = 1.0);
      bool removeEdge(g2o::SparseOptimizer::Edge* e) { return m_optimizer.removeEdge(e); }

      /** Optimize and return the actual no of iterations performed */
      int optimize(int max_no_of_iterations = 100);
      /** Optimize frameId with its [1..noOfRings] and noOfRings+1 order neighbours, but keeping fixed the latter (RBA)
        * The optimized subset is written to vSetPtr and the fixed subset to fSetPtr if not NULL and empty;
        * if not empty, the given vertices are used and no RBA subset is computed
        */
      int relativeOptimization(int frameId, unsigned noOfRings, int max_no_of_iterations = 100,
                                g2o::HyperGraph::VertexSet* vSetPtr = NULL, g2o::HyperGraph::VertexSet* fSetPtr = NULL);
      /** Create the RBA subset centered in vx #frameId and get the poses */
      bool getRBASubsetState(int frameId, unsigned noOfRings, PoseIsometryMap& poses,
                              g2o::HyperGraph::VertexSet* vSetPtr = NULL, g2o::HyperGraph::VertexSet* fSetPtr = NULL);

      /** Optimizer stat */
      inline double chi2() const { return m_optimizer.activeChi2(); }
      inline double weightedMeanChi2() const { assert(m_edge_weights > 0); return chi2()/m_edge_weights; }
      inline unsigned getNoOfVertices() const { return m_optimizer.vertices().size(); }
      inline unsigned getNoOfEdges() const { return m_optimizer.edges().size(); }

      /** Get the result */
      void getOptimizedPoses(PoseIsometryMap& poses) const;

      inline const g2o::SparseOptimizer& getGraph() const { return m_optimizer; }
      inline g2o::SparseOptimizer& getGraph() { return m_optimizer; }
      inline void clear() { m_optimizer.clear(); m_edge_weights = 0.; }

    protected:
      g2o::SparseOptimizer::Vertex* getPose(int pose_id, bool create);
      const g2o::SparseOptimizer::Vertex* getPose(int pose_id) const;
      void createRBASubset(g2o::HyperGraph::Vertex* origin, unsigned ring, g2o::HyperGraph::VertexSet& vset, g2o::HyperGraph::VertexSet& fixedset);
      void collectActiveNeighbourhood(g2o::HyperGraph::Vertex* origin, const g2o::HyperGraph::VertexSet& refset, g2o::HyperGraph::VertexSet& vset);

      g2o::SparseOptimizer m_optimizer;
      ConvergenceCheckAction m_action;
      double m_edge_weights;
  };

}

#endif // SLAM_DUNK_GRAPH_BACKEND_H
