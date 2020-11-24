#include <iostream>
#include <glk/thin_lines.hpp>
#include <glk/ext/graph/factor_graph.hpp>

namespace glk {
namespace ext {

void FactorGraph::add_variable(const std::shared_ptr<Variable>& variable) {
  auto found = variables.find(variable->id);
  if(found != variables.end()) {
    std::cerr << "warning: variable " << variable->id << " already exists in the grpah!!" << std::endl;
  }

  variables.insert(found, std::make_pair(variable->id, variable));
}

void FactorGraph::add_factor(const std::shared_ptr<Factor>& factor) {
  factors.push_back(factor);
}

std::shared_ptr<FactorGraph::Variable> FactorGraph::variable(long id) const {
  auto found = variables.find(id);
  if(found == variables.end()) {
    return nullptr;
  }

  return found->second;
}

void FactorGraph::draw(glk::GLSLShader& shader) const {
  for(const auto& variable : variables) {
    variable.second->draw(shader);
  }

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lines;
  for(const auto& factor : factors) {
    factor->draw(shader);
    factor->append_lines(lines);
  }

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("material_color", Eigen::Vector4f(0.0, 1.0f, 0.0, 1.0f));
  shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  glk::ThinLines factor_lines(lines);
  factor_lines.draw(shader);
}
}  // namespace ext
}  // namespace glk