#include <glk/indexed_pointcloud_buffer.hpp>

namespace glk {

IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const unsigned int* indices, int num_indices)
: cloud_buffer(cloud_buffer),
  num_indices(num_indices),
  ebo(0) {
  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, indices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<unsigned int>& indices)
: IndexedPointCloudBuffer(cloud_buffer, indices.data(), indices.size()) {}

template <typename Integral>
IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const Integral* indices, int num_indices)
: IndexedPointCloudBuffer(cloud_buffer, std::vector<unsigned int>(indices, indices + num_indices)) {}

template <typename Integral>
IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<Integral>& indices)
: IndexedPointCloudBuffer(cloud_buffer, std::vector<unsigned int>(indices.begin(), indices.end())) {}

template <>
IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const int* indices, int num_indices)
: IndexedPointCloudBuffer(cloud_buffer, reinterpret_cast<const unsigned int*>(indices), num_indices) {}
template <>
IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<int>& indices)
: IndexedPointCloudBuffer(cloud_buffer, reinterpret_cast<const unsigned int*>(indices.data()), indices.size()) {}
template IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::int64_t* indices, int num_indices);
template IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<std::int64_t>& indices);
template IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::uint64_t* indices, int num_indices);
template IndexedPointCloudBuffer::IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<std::uint64_t>& indices);

IndexedPointCloudBuffer::~IndexedPointCloudBuffer() {
  if (ebo) {
    glDeleteBuffers(1, &ebo);
  }
}

void IndexedPointCloudBuffer::draw(glk::GLSLShader& shader) const {
  cloud_buffer->bind(shader);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_POINTS, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  cloud_buffer->unbind(shader);
}

GLuint IndexedPointCloudBuffer::ebo_id() const {
  return ebo;
}
}  // namespace glk