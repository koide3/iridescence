#include <glk/thin_lines.hpp>

#include <iostream>
#include <Eigen/Geometry>

namespace glk {

ThinLines::ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices)
    : num_vertices(vertices.size()) {
  vao = vbo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ThinLines::~ThinLines() {
  glDeleteBuffers(1, &vbo);
  glDeleteVertexArrays(1, &vao);
}

void ThinLines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glDrawArrays(GL_LINES, 0, num_vertices);

  glDisableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace glk