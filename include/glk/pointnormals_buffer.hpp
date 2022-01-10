#ifndef GLK_POINT_NORMALS_BUFFER_HPP
#define GLK_POINT_NORMALS_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawable.hpp>
#include <glk/thin_lines.hpp>

namespace glk {

class PointNormalsBuffer : public glk::Drawable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointNormalsBuffer>;

  PointNormalsBuffer(const float* vertices, size_t vertex_stride, const float* normals, size_t normal_stride, int num_points, double normal_length);

  template <template <class> class Allocator>
  PointNormalsBuffer(
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& normals,
    double normal_length);

  virtual ~PointNormalsBuffer() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::ThinLines> normal_lines;
};

}  // namespace glk

#endif