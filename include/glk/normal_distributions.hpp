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
  /// @brief Constructor
  /// @param means       Means (4D vector). The 4th element is ignored.
  /// @param covs        Covariances (4x4 matrix). The last row and column are ignored.
  /// @param num_points  Number of normal distributions to render.
  /// @param sigma       Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
  NormalDistributions(const float* means, const float* covs, int num_points, float sigma = 1.0f);

  /// @brief Constructor
  /// @note  Only T = float/double and D = 3/4 are supported. For D = 4, the 4th element of means and the last row and column of covs are ignored.
  /// @param means       Means
  /// @param covs        Covariances
  /// @param num_points  Number of normal distributions to render.
  /// @param sigma       Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
  template <typename T, int D>
  NormalDistributions(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float sigma = 1.0f);

  /// @brief Constructor
  /// @note  Only T = float/double and D = 3/4 are supported. For D = 4, the 4th element of means and the last row and column of covs are ignored.
  /// @param means       Means
  /// @param covs        Covariances (must have the same size as means)
  /// @param sigma       Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
  template <typename T, int D, template <class> class Allocator>
  NormalDistributions(
    const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
    const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
    float sigma = 1.0f)
  : NormalDistributions(means.data(), covs.data(), means.size(), sigma) {}

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

/// @brief Old CPU implementation of NormalDistributions (slower). The arguments are the same as NormalDistributions.
class NormalDistributions_old : public glk::Drawable {
public:
  template <typename T, int D>
  NormalDistributions_old(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float sigma = 1.0f);

  template <typename T, int D, template <class> class Allocator>
  NormalDistributions_old(
    const std::vector<Eigen::Matrix<T, D, 1>, Allocator<Eigen::Matrix<T, D, 1>>>& means,
    const std::vector<Eigen::Matrix<T, D, D>, Allocator<Eigen::Matrix<T, D, D>>>& covs,
    float sigma = 1.0f)
  : NormalDistributions_old(means.data(), covs.data(), means.size(), sigma) {}

  virtual ~NormalDistributions_old();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::Mesh> mesh;
};

}  // namespace glk

#endif