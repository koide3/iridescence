#ifndef GLK_EXT_FACTOR_GRAPH_G2O_HPP
#define GLK_EXT_FACTOR_GRAPH_G2O_HPP

#include <glk/ext/graph/factor_graph.hpp>
#include <g2o/core/optimizable_graph.h>

namespace glk {
namespace ext {

void load_graph_g2o(FactorGraph& graph, const g2o::OptimizableGraph& graph_g2o);

}
}  // namespace glk

#endif