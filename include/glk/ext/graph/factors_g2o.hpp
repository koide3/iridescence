#ifndef GLK_EXT_FACTORS_G2O_HPP
#define GLK_EXT_FACTORS_G2O_HPP

#include <glk/ext/graph/variable.hpp>

#include <glk/primitives/primitives.hpp>
#include <g2o/types/slam3d/types_slam3d.h>

namespace glk {
namespace ext {

class VariableSE3_g2o : public FactorGraph::Variable {
public:
  VariableSE3_g2o(const g2o::VertexSE3* v) : Variable(v->id()), v(v) {}
  virtual ~VariableSE3_g2o() {}

  virtual void draw(glk::GLSLShader& shader) const override {
    shader.set_uniform("color_mode", 2);
    shader.set_uniform("model_matrix", v->estimate().cast<float>().matrix());
    glk::Primitives::primitive(glk::Primitives::COORDINATE_SYSTEM).draw(shader);
  }

  virtual Eigen::Vector3f representative_point() const {
    return v->estimate().translation().cast<float>();
  }

public:
  const g2o::VertexSE3* v;
};

class FactorSE3_g2o : public FactorGraph::Factor {
public:
  FactorSE3_g2o(const FactorGraph& graph, const g2o::EdgeSE3* e) : e(e) {
    auto v1 = graph.variable(e->vertices()[0]->id());
    auto v2 = graph.variable(e->vertices()[1]->id());
    if(v1 == nullptr || v2 == nullptr) {
      std::cerr << "warning: vertices " << e->vertices()[0]->id() << " " << e->vertices()[1]->id() << " not found!!" << std::endl;
      return;
    }

    this->variables.push_back(v1.get());
    this->variables.push_back(v2.get());
  }

public:
  const g2o::EdgeSE3* e;
};

template<typename G2O_T, typename T>
std::shared_ptr<FactorGraph::Variable> construct_variable(g2o::HyperGraph::Vertex* v) {
  const G2O_T* v_ = dynamic_cast<const G2O_T*>(v);
  if(v_ == nullptr) {
    return nullptr;
  }
  return std::make_shared<T>(v_);
}

std::shared_ptr<FactorGraph::Variable> construct_variable_from_g2o(g2o::HyperGraph::Vertex* v) {
  return construct_variable<g2o::VertexSE3, VariableSE3_g2o>(v);
}

template<typename G2O_T, typename T>
std::shared_ptr<FactorGraph::Factor> construct_factor(const FactorGraph& graph, g2o::HyperGraph::Edge* e) {
  const G2O_T* e_ = dynamic_cast<const G2O_T*>(e_);
  if(e_ == nullptr) {
    return nullptr;
  }
  return std::make_shared<T>(graph, e_);
}

std::shared_ptr<FactorGraph::Factor> construct_factor_from_g2o(const FactorGraph& graph, g2o::HyperGraph::Edge* e) {
  return construct_factor<g2o::EdgeSE3, FactorSE3_g2o>(graph, e);
}

}  // namespace ext
}  // namespace glk

#endif