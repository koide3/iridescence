#include <glk/ext/graph/factor_graph_g2o.hpp>

#include <glk/ext/graph/factors_g2o.hpp>

namespace glk {
namespace ext {

void load_graph_g2o(FactorGraph& graph, const g2o::OptimizableGraph& graph_g2o) {
  for(const auto& vertex: graph_g2o.vertices()) {
    auto v = construct_variable_from_g2o(vertex.second);
    if(v) {
      graph.add_variable(v);
    }
  }

  for(const auto& edge : graph_g2o.edges()) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);
    if(e) {
      graph.add_factor(std::make_shared<FactorSE3_g2o>(graph, e));
    }
  }
}

}
}  // namespace glk