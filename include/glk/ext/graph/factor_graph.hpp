#ifndef GLK_FACTOR_GRAPH_HPP
#define GLK_FACTOR_GRAPH_HPP

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glk/drawble.hpp>

namespace glk {
namespace ext {

class FactorGraph : public glk::Drawable {
public:
  class Variable;
  class Factor;

  FactorGraph() {}
  virtual ~FactorGraph() {}

  void add_variable(const std::shared_ptr<Variable>& variable);
  void add_factor(const std::shared_ptr<Factor>& factor);

  std::shared_ptr<Variable> variable(long id) const;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unordered_map<long, std::shared_ptr<Variable>> variables;
  std::vector<std::shared_ptr<Factor>> factors;
};
}  // namespace ext
}  // namespace glk

#include <glk/ext/graph/variable.hpp>
#include <glk/ext/graph/factor.hpp>

#endif