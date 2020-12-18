#ifndef GLK_FACTOR_GRAPH_VARIABLE_HPP
#define GLK_FACTOR_GRAPH_VARIABLE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glk/primitives/primitives.hpp>
#include <glk/ext/graph/factor_graph.hpp>

namespace glk {
namespace ext {

class FactorGraph::Variable : public glk::Drawable {
public:
  Variable(long id) : id(id) {}
  virtual ~Variable() {}

  virtual void draw(glk::GLSLShader& shader) const override {}
  virtual Eigen::Vector3f representative_point() const = 0;

public:
  long id;
};

class VariableSE3 : public FactorGraph::Variable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VariableSE3(long id, const Eigen::Isometry3d& pose) : Variable(id), pose(pose) {}
  virtual ~VariableSE3() {}

  virtual void draw(glk::GLSLShader& shader) {
    shader.set_uniform("color_mode", 2);
    shader.set_uniform("model_matrix", pose.cast<float>().matrix());
    glk::Primitives::primitive(glk::Primitives::COORDINATE_SYSTEM).draw(shader);
  }

  virtual Eigen::Vector3f representative_point() const {
    return pose.translation().cast<float>();
  }

private:
  Eigen::Isometry3d pose;
};

}  // namespace ext
}  // namespace glk

#endif