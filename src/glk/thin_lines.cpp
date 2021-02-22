#include <glk/thin_lines.hpp>

#include <iostream>
#include <Eigen/Geometry>

namespace glk {

ThinLines::ThinLines(const float* vertices, int num_vertices, bool line_strip)
: ThinLines(vertices, nullptr, num_vertices, line_strip)
{}

ThinLines::ThinLines(const float* vertices, const float* colors, int num_vertices, bool line_strip) {
  this->num_vertices = num_vertices;
  this->mode = line_strip ? GL_LINE_STRIP : GL_LINES;

  vao = vbo = cbo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * num_vertices, vertices, GL_STATIC_DRAW);

  if(colors) {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * num_vertices, colors, GL_STATIC_DRAW);
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ThinLines::ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, bool line_strip) {
  num_vertices = vertices.size();
  mode = line_strip ? GL_LINE_STRIP : GL_LINES;

  vao = vbo = cbo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ThinLines::ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors, bool line_strip) {
  num_vertices = vertices.size();
  mode = line_strip ? GL_LINE_STRIP : GL_LINES;

  vao = vbo = cbo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

  glGenBuffers(1, &cbo);
  glBindBuffer(GL_ARRAY_BUFFER, cbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors.size() * 4, colors.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ThinLines::~ThinLines() {
  if(cbo) {
    glDeleteBuffers(1, &cbo);
  }
  glDeleteBuffers(1, &vbo);
  glDeleteVertexArrays(1, &vao);
}

void ThinLines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint color_loc = shader.attrib("vert_color");

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if(cbo) {
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  }

  glDrawArrays(mode, 0, num_vertices);

  glDisableVertexAttribArray(position_loc);

  if(cbo) {
    glDisableVertexAttribArray(color_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace glk