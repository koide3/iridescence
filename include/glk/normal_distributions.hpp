#ifndef GLK_NORMAL_DISTRIBUTIONS_HPP
#define GLK_NORMAL_DISTRIBUTIONS_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace glk {

class Mesh;

class NormalDistributions : public glk::Drawable {
public:
  NormalDistributions(const Eigen::Vector3f* means, const Eigen::Matrix3f* covs, int num_points, float scale = 1.0f);

  NormalDistributions(const Eigen::Matrix<float, 3, -1>& means, const Eigen::Matrix<float, 9, -1>& covs, float scale = 1.0f);
  NormalDistributions(const Eigen::Matrix<double, 3, -1>& means, const Eigen::Matrix<double, 9, -1>& covs, float scale = 1.0f);

  template <typename T, int D, template <class> class Allocator>
  NormalDistributions(
    const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
    const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
    float scale = 1.0f);

  virtual ~NormalDistributions();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};

template <typename T, int D, template <class> class Allocator>
NormalDistributions::NormalDistributions(
  const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
  const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
  float scale)
: NormalDistributions(
    Eigen::Matrix<T, 3, -1>(Eigen::Map<const Eigen::Matrix<T, D, -1>>(means.front().data(), D, means.size()).template topRows<3>()),
    Eigen::Matrix<T, 9, -1>(Eigen::Map<const Eigen::Matrix<T, D * D, -1>>(covs.front().data(), D * D, covs.size()).template topRows<9>()),
    scale) {}

}  // namespace glk

#endif