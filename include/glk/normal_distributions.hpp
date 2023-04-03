#ifndef GLK_NORMAL_DISTRIBUTIONS_HPP
#define GLK_NORMAL_DISTRIBUTIONS_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>
#include <glk/type_conversion.hpp>

namespace glk {

class Mesh;

/**
 * @brief A drawable class to render a set of normal distributions as ellipsoids.
 *        This class efficiently perform ellipsoids mesh generation using GLSL
 *        compute shader and is much faster than the old implementation.
 */
class NormalDistributions : public glk::Drawable {
public:
  NormalDistributions(const float* means, const float* covs, int num_points, float scale = 1.0f);

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
  static std::shared_ptr<glk::GLSLShader> vertices_shader;
  static std::shared_ptr<glk::GLSLShader> indices_shader;

  int num_indices;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
};

class NormalDistributions_old : public glk::Drawable {
public:
  template <typename T, int D>
  NormalDistributions_old(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float scale = 1.0f);

  template <typename T, int D, template <class> class Allocator>
  NormalDistributions_old(
    const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
    const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
    float scale = 1.0f)
  : NormalDistributions_old(means.data(), covs.data(), means.size(), scale) {}

  virtual ~NormalDistributions_old();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};

}  // namespace glk

#endif