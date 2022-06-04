#ifndef GLK_NORMAL_DISTRIBUTIONS_HPP
#define GLK_NORMAL_DISTRIBUTIONS_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace glk {

class Mesh;

class NormalDistributions : public glk::Drawable {
public:
  template <typename T, int D>
  NormalDistributions(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float scale = 1.0f);

  template <typename T, int D, template <class> class Allocator>
  NormalDistributions(
    const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
    const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
    float scale = 1.0f)
  : NormalDistributions(means.data(), covs.data(), means.size(), scale) {}

  virtual ~NormalDistributions();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};

}  // namespace glk

#endif