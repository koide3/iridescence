#include <glk/pointcloud_buffer.hpp>

namespace glk {

PointCloudBuffer::PointCloudBuffer(int stride, int num_points) {
  this->stride = stride;
  this->num_points = num_points;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, nullptr, GL_STATIC_DRAW);
}

PointCloudBuffer::PointCloudBuffer(const float* data, int stride, int num_points) {
  this->stride = stride;
  this->num_points = num_points;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, data, GL_STATIC_DRAW);
}

PointCloudBuffer::~PointCloudBuffer() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) const {
  if(num_points == 0) {
    return;
  }

  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);

  glDrawArrays(GL_POINTS, 0, num_points);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
}

}  // namespace glk
