#ifndef GLK_NORMAL_DISTRIBUTIONS_HPP
#define GLK_NORMAL_DISTRIBUTIONS_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace glk {

class Mesh;

class NormalDistributions : public glk::Drawable {
public:
  template <template <class> class Allocator>
  NormalDistributions(
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& means,
    const std::vector<Eigen::Matrix3f, Allocator<Eigen::Matrix3f>>& covs,
    float scale = 1.0f);

  template <template <class> class Allocator>
  NormalDistributions(
    const std::vector<Eigen::Vector3d, Allocator<Eigen::Vector3d>>& means,
    const std::vector<Eigen::Matrix3d, Allocator<Eigen::Matrix3d>>& covs,
    float scale = 1.0f);

  virtual ~NormalDistributions();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};
}  // namespace glk

#endif