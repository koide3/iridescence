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