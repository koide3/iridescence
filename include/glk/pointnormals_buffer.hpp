#ifndef GLK_POINT_NORMALS_BUFFER_HPP
#define GLK_POINT_NORMALS_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawble.hpp>
#include <glk/thin_lines.hpp>

namespace glk {

class PointNormalsBuffer : public glk::Drawable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointNormalsBuffer>;
  using Points = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;

  PointNormalsBuffer(const float* vertices, size_t vertex_stride, const float* normals, size_t normal_stride, int num_points, double normal_length);
  PointNormalsBuffer(const Points& vertices, const Points& normals, double normal_length);
  virtual ~PointNormalsBuffer() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::ThinLines> normal_lines;
};

}  // namespace glk

#endif