
#include "slamdunk/graph_utils.h"
#include "slamdunk/graph_backend.h"
#include <sstream>

std::string slamdunk::graphToDot(const GraphBackend* gb, int root_id,
                              const g2o::HyperGraph::VertexSet& colored,
                              const g2o::HyperGraph::VertexSet& highlighted,
                              const std::string& graph_name)
{
  double buf[7];
  std::ostringstream sstream;
  sstream << "digraph " << graph_name << " { root=" << root_id << ";";

  for(g2o::HyperGraph::VertexSet::const_iterator vit = colored.begin(); vit != colored.end(); ++vit)
  {
    static_cast<const g2o::OptimizableGraph::Vertex*>(*vit)->getEstimateData(buf);
    sstream << " " << (*vit)->id() << " [style=filled,fillcolor=" << ((*vit)->id() == root_id ? "yellow" : "orange")
                              //<< ",pos=\"" << (int)(buf[0]*1000) << "," << (int)(buf[2]*1000) << "!\"];";
                              << "];";
  }

  for(g2o::HyperGraph::VertexSet::const_iterator vit = highlighted.begin(); vit != highlighted.end(); ++vit)
    if(!colored.count(*vit))
    {
      static_cast<const g2o::OptimizableGraph::Vertex*>(*vit)->getEstimateData(buf);
      sstream << " " << (*vit)->id() << " [style=filled,fillcolor=" << ((*vit)->id() == root_id ? "yellow" : "lightblue")
                              //<< ",pos=\"" << (int)(buf[0]*1000) << "," << (int)(buf[2]*1000) << "!\"];";
                              << "];";
    }

  std::set< std::pair<int, int> > edges;
  const g2o::SparseOptimizer::EdgeSet& eset = gb->getGraph().edges();
  for(g2o::SparseOptimizer::EdgeSet::const_iterator eit = eset.begin(); eit != eset.end(); ++eit)
    if(edges.insert(std::make_pair((*eit)->vertices()[0]->id(), (*eit)->vertices()[1]->id())).second)
      sstream << " " << (*eit)->vertices()[0]->id() << " -> " << (*eit)->vertices()[1]->id() << ";";

  sstream << " }" << std::endl;
  return sstream.str();
}
