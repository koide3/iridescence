#include <glk/pointcloud_buffer.hpp>

#include <iostream>

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
  for(const auto& aux : aux_buffers) {
    glDeleteBuffers(1, &aux.buffer);
  }
  glDeleteBuffers(1, &vbo);
}

void PointCloudBuffer::add_color(const float* data, int stride, int num_points) {
  add_buffer("vert_color", 4, data, stride, num_points);
}

void PointCloudBuffer::add_buffer(const std::string& attribute_name, int dim, const float* data, int stride, int num_points) {
  assert(this->num_points == num_points);

  glBindVertexArray(vao);

  GLuint buffer_id;
  glGenBuffers(1, &buffer_id);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, data, GL_STATIC_DRAW);

  aux_buffers.push_back(AuxBufferData{attribute_name, dim, stride, buffer_id});
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) const {
  if(num_points == 0) {
    return;
  }

  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, stride, 0);

  for(const auto& aux : aux_buffers) {
    GLint attrib_loc = shader.attrib(aux.attribute_name);
    glEnableVertexAttribArray(attrib_loc);
    glBindBuffer(GL_ARRAY_BUFFER, aux.buffer);
    glVertexAttribPointer(attrib_loc, aux.dim, GL_FLOAT, GL_FALSE, aux.stride, 0);
  }

  glDrawArrays(GL_POINTS, 0, num_points);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(position_loc);
  for(const auto& aux : aux_buffers) {
    glDisableVertexAttribArray(shader.attrib(aux.attribute_name));
  }
}

GLuint PointCloudBuffer::vba_id() const {
  return vao;
}
GLuint PointCloudBuffer::vbo_id() const {
  return vbo;
}

int PointCloudBuffer::get_aux_size() const {
  return aux_buffers.size();
}

const AuxBufferData& PointCloudBuffer::get_aux_buffer(int i) const {
  return aux_buffers[i];
}

}  // namespace glk
