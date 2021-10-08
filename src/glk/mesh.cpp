#include <glk/mesh.hpp>

#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

Mesh::Mesh(const void* vertices, int vertex_stride, const void* normals, int normal_stride, int num_vertices, const void* indices, int num_indices, bool wireframe) {
  this->wireframe = wireframe;
  this->num_vertices = num_vertices;
  this->num_indices = num_indices;
  this->vertex_stride = vertex_stride;
  this->normal_stride = normal_stride;

  vbo = nbo = ebo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_stride * num_vertices, vertices, GL_STATIC_DRAW);

  if(normals) {
    glGenBuffers(1, &nbo);
    glBindBuffer(GL_ARRAY_BUFFER, nbo);
    glBufferData(GL_ARRAY_BUFFER, normal_stride * num_vertices, normals, GL_STATIC_DRAW);
  }

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, indices, GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Mesh::Mesh(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& normals, const std::vector<int>& indices, bool wireframe)
    : Mesh(vertices.data(), sizeof(float) * 3, normals.data(), sizeof(float) * 3, vertices.size(), indices.data(), indices.size(), wireframe) {}

Mesh ::~Mesh() {
  glDeleteBuffers(1, &vbo);
  if(nbo) {
    glDeleteBuffers(1, &nbo);
  }
  glDeleteBuffers(1, &ebo);
  glDeleteBuffers(1, &vao);
}

void Mesh::draw(glk::GLSLShader& shader) const {
  if(wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }

  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, vertex_stride, 0);

  GLint normal_loc = -1;
  if(nbo) {
    normal_loc = shader.attrib("vert_normal");
    glEnableVertexAttribArray(normal_loc);
    glBindBuffer(GL_ARRAY_BUFFER, nbo);
    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE, normal_stride, 0);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(position_loc);

  if(nbo) {
    glDisableVertexAttribArray(normal_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  if(wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
}

}  // namespace glk