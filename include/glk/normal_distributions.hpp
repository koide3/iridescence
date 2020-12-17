#ifndef GLK_NORMAL_DISTRIBUTIONS_HPP
#define GLK_NORMAL_DISTRIBUTIONS_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawble.hpp>

namespace glk {

class Mesh;

class NormalDistributions : public glk::Drawable {
public:
  NormalDistributions(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& means, const std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs, float scale = 1.0f);
  virtual ~NormalDistributions();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};
}  // namespace glk

#endif