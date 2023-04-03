#include <glk/thin_lines.hpp>

#include <iostream>
#include <Eigen/Geometry>

#include <glk/async_buffer_copy.hpp>

namespace glk {

ThinLines::ThinLines(const float* vertices, int num_vertices, bool line_strip) : ThinLines(vertices, nullptr, num_vertices, nullptr, 0, line_strip) {}

ThinLines::ThinLines(const float* vertices, const float* colors, int num_vertices, bool line_strip) : ThinLines(vertices, colors, num_vertices, nullptr, 0, line_strip) {}

ThinLines::ThinLines(const float* vertices, const float* colors, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip) : line_width(1.0f) {
  this->num_vertices = num_vertices;
  this->num_indices = num_indices;
  this->mode = line_strip ? GL_LINE_STRIP : GL_LINES;

  vao = vbo = cbo = ebo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * num_vertices, nullptr, GL_STATIC_DRAW);
  write_buffer_async(GL_ARRAY_BUFFER, sizeof(float) * 3 * num_vertices, vertices);

  if (colors) {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * num_vertices, nullptr, GL_STATIC_DRAW);
    write_buffer_async(GL_ARRAY_BUFFER, sizeof(float) * 4 * num_vertices, colors);
  }

  if (indices) {
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, nullptr, GL_STATIC_DRAW);
    write_buffer_async(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, indices);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

ThinLines::~ThinLines() {
  if (cbo) {
    glDeleteBuffers(1, &cbo);
  }
  if (ebo) {
    glDeleteBuffers(1, &ebo);
  }

  glDeleteBuffers(1, &vbo);
  glDeleteVertexArrays(1, &vao);
}

void ThinLines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint color_loc = shader.attrib("vert_color");

  glLineWidth(line_width);

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if (cbo) {
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  }

  if (!ebo) {
    glDrawArrays(mode, 0, num_vertices);
  } else {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glDrawElements(mode, num_indices, GL_UNSIGNED_INT, nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  glDisableVertexAttribArray(position_loc);

  if (cbo) {
    glDisableVertexAttribArray(color_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace glk