#include <glk/lines.hpp>

#include <iostream>
#include <Eigen/Geometry>

namespace glk {

Lines::Lines(float line_width, const Eigen::Vector3f* vertices_, const Eigen::Vector4f* colors_, const Eigen::Vector4i* infos_, int num_points, bool line_strip) {
  vao = vbo = cbo = ibo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector4f> colors;
  std::vector<Eigen::Vector4i> infos;
  if (!line_strip) {
    vertices.assign(vertices_, vertices_ + num_points);
    if (colors_) {
      colors.assign(colors_, colors_ + num_points);
    }
    if (infos_) {
      infos.assign(infos_, infos_ + num_points);
    }
  } else {
    for (int i = 1; i < num_points; i++) {
      vertices.push_back(vertices_[i - 1]);
      vertices.push_back(vertices_[i]);

      if (colors_) {
        colors.push_back(colors_[i - 1]);
        colors.push_back(colors_[i]);
      }

      if (infos_) {
        infos.push_back(infos_[i - 1]);
        infos.push_back(infos_[i]);
      }
    }
  }

  num_vertices = vertices.size();

  std::vector<Eigen::Vector3f> vertices_ext(vertices.size() * 4);
  for (int i = 0; i < vertices.size(); i += 2) {
    Eigen::Vector3f direction = vertices[i + 1] - vertices[i];
    Eigen::Vector3f axis = std::abs(direction.normalized().dot(Eigen::Vector3f::UnitZ())) < 0.9f ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();

    Eigen::Vector3f x = axis.cross(direction).normalized();
    Eigen::Vector3f y = x.cross(direction).normalized();

    vertices_ext[i * 4] = vertices[i] - x * line_width;
    vertices_ext[i * 4 + 1] = vertices[i + 1] - x * line_width;
    vertices_ext[i * 4 + 2] = vertices[i] + x * line_width;
    vertices_ext[i * 4 + 3] = vertices[i + 1] + x * line_width;

    vertices_ext[i * 4 + 4] = vertices[i] - y * line_width;
    vertices_ext[i * 4 + 5] = vertices[i + 1] - y * line_width;
    vertices_ext[i * 4 + 6] = vertices[i] + y * line_width;
    vertices_ext[i * 4 + 7] = vertices[i + 1] + y * line_width;
  }

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices_ext.size() * 3, vertices_ext.data(), GL_STATIC_DRAW);

  if (!colors.empty()) {
    std::vector<Eigen::Vector4f> colors_ext(colors.size() * 4);
    for (int i = 0; i < colors.size(); i += 2) {
      for (int j = 0; j < 4; j++) {
        colors_ext[i * 4 + j * 2] = colors[i];
        colors_ext[i * 4 + j * 2 + 1] = colors[i + 1];
      }
    }
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors_ext.size() * 4, colors_ext.data(), GL_STATIC_DRAW);
  }

  if (!infos.empty()) {
    std::vector<Eigen::Vector4i> infos_ext(infos.size() * 4);
    for (int i = 0; i < infos.size(); i += 2) {
      for (int j = 0; j < 4; j++) {
        infos_ext[i * 4 + j * 2] = infos[i];
        infos_ext[i * 4 + j * 2 + 1] = infos[i + 1];
      }
    }
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(int) * infos_ext.size() * 4, infos_ext.data(), GL_STATIC_DRAW);
  }

  /*
  std::vector<int> indices;
  for(int i = 0; i < vertices_ext.size(); i += 4) {
    indices.push_back(i);
    indices.push_back(i + 1);
    indices.push_back(i + 2);
    indices.push_back(i + 1);
    indices.push_back(i + 3);
    indices.push_back(i + 2);
  }
  */

  std::vector<int> sub_indices = {0, 1, 4, 1, 5, 4, 4, 5, 2, 5, 3, 2, 2, 3, 6, 3, 6, 7, 6, 7, 0, 7, 1, 0};

  std::vector<int> indices;
  for (int i = 0; i < vertices_ext.size(); i += 8) {
    for (int j = 0; j < sub_indices.size(); j++) {
      indices.push_back(sub_indices[j] + i);
    }
  }
  num_indices = indices.size();

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Lines::Lines(float line_width, const Eigen::Vector3f* vertices, int num_points, bool line_strip) : Lines(line_width, vertices, nullptr, nullptr, num_points, line_strip) {}

Lines::Lines(float line_width, const Eigen::Vector3f* vertices, const Eigen::Vector4f* colors, int num_points, bool line_strip)
: Lines(line_width, vertices, colors, nullptr, num_points, line_strip) {}

Lines::~Lines() {
  glDeleteBuffers(1, &vbo);
  if (cbo) {
    glDeleteBuffers(1, &cbo);
  }
  if (ibo) {
    glDeleteBuffers(1, &ibo);
  }

  glDeleteBuffers(1, &ebo);
  glDeleteVertexArrays(1, &vao);
}

void Lines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint color_loc = 0;
  GLint info_loc = 0;

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if (cbo) {
    color_loc = shader.attrib("vert_color");
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  }

  if (ibo) {
    info_loc = shader.attrib("vert_info");
    glEnableVertexAttribArray(info_loc);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glVertexAttribIPointer(info_loc, 4, GL_INT, 0, 0);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(position_loc);

  if (cbo) {
    glDisableVertexAttribArray(color_loc);
  }
  if (ibo) {
    glDisableVertexAttribArray(info_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace glk