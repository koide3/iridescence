#ifndef GLK_FACTOR_GRAPH_FACTOR_HPP
#define GLK_FACTOR_GRAPH_FACTOR_HPP

#include <glk/ext/graph/factor_graph.hpp>

namespace glk {
namespace ext {

class FactorGraph::Factor : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<Factor>;

  Factor() {}
  Factor(Variable* v1) : variables({v1}) {}
  Factor(Variable* v1, Variable* v2) : variables({v1, v2}) {}
  virtual ~Factor() {}

  virtual void draw(glk::GLSLShader& shader) const override {}
  virtual void append_lines(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& lines) const {
    if(variables.size() < 2) {
      return;
    }

    if(variables.size() == 2) {
      lines.push_back(variables[0]->representative_point());
      lines.push_back(variables[1]->representative_point());
      return;
    }

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& variable : variables) {
      centroid += variable->representative_point();
    }
    centroid /= variables.size();

    for(const auto& variable : variables) {
      lines.push_back(centroid);
      lines.push_back(variable->representative_point());
    }
  }

protected:
  std::vector<Variable*> variables;
};

}  // namespace ext
}  // namespace glk

#endif