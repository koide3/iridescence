#include <glk/pointnormals_buffer.hpp>

#include <iostream>

namespace glk {

PointNormalsBuffer::PointNormalsBuffer(const float* vertices, size_t vertex_stride, const float* normals, size_t normal_stride, int num_points, double normal_length) {
  std::vector<Eigen::Vector3f> lines(num_points * 2);

  for (int i = 0; i < num_points; i++) {
    lines[i * 2] = Eigen::Map<const Eigen::Vector3f>(vertices + i * vertex_stride / sizeof(float));
    lines[i * 2 + 1] = lines[i * 2] + Eigen::Map<const Eigen::Vector3f>(normals + i * normal_stride / sizeof(float)) * normal_length;
  }

  normal_lines.reset(new ThinLines(lines));
}

template <template <class> class Allocator>
PointNormalsBuffer::PointNormalsBuffer(
  const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
  const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& normals,
  double normal_length)
: PointNormalsBuffer(vertices[0].data(), sizeof(Eigen::Vector3f), normals[0].data(), sizeof(Eigen::Vector3f), vertices.size(), normal_length) {}

template PointNormalsBuffer::PointNormalsBuffer(const std::vector<Eigen::Vector3f>& vertices, const std::vector<Eigen::Vector3f>& normals, double normal_length);

PointNormalsBuffer::~PointNormalsBuffer() {}

void PointNormalsBuffer::draw(glk::GLSLShader& shader) const {
  normal_lines->draw(shader);
}

}  // namespace glk