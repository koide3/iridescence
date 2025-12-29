#include <glk/mesh.hpp>

#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/texture.hpp>
#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

Mesh::Mesh(
  const void* vertices,
  int vertex_stride,
  const void* normals,
  int normal_stride,
  const void* colors,
  int color_stride,
  const void* tex_coords,
  int tex_coord_stride,
  int num_vertices,
  const void* indices,
  int num_indices,
  bool wireframe) {
  //
  this->line_width = 1.0f;
  this->wireframe = wireframe;
  this->num_vertices = num_vertices;
  this->num_indices = num_indices;
  this->vertex_stride = vertex_stride;
  this->normal_stride = normal_stride;
  this->color_stride = color_stride;
  this->tex_coord_stride = tex_coord_stride;
  this->texture_target = GL_TEXTURE1;

  vbo = nbo = cbo = tbo = ebo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_stride * num_vertices, vertices, GL_STATIC_DRAW);

  if (normals) {
    glGenBuffers(1, &nbo);
    glBindBuffer(GL_ARRAY_BUFFER, nbo);
    glBufferData(GL_ARRAY_BUFFER, normal_stride * num_vertices, normals, GL_STATIC_DRAW);
  }

  if (colors) {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, color_stride * num_vertices, colors, GL_STATIC_DRAW);
  }

  if (tex_coords) {
    glGenBuffers(1, &tbo);
    glBindBuffer(GL_ARRAY_BUFFER, tbo);
    glBufferData(GL_ARRAY_BUFFER, tex_coord_stride * num_vertices, tex_coords, GL_STATIC_DRAW);
  }

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, indices, GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Mesh::Mesh(const void* vertices, int vertex_stride, const void* normals, int normal_stride, int num_vertices, const void* indices, int num_indices, bool wireframe)
: Mesh(vertices, vertex_stride, normals, normal_stride, nullptr, 0, num_vertices, indices, num_indices, wireframe) {}

Mesh::Mesh(
  const void* vertices,
  int vertex_stride,
  const void* normals,
  int normal_stride,
  const void* colors,
  int color_stride,
  int num_vertices,
  const void* indices,
  int num_indices,
  bool wireframe)
: Mesh(vertices, vertex_stride, normals, normal_stride, colors, color_stride, nullptr, 0, num_vertices, indices, num_indices, wireframe) {}

Mesh ::~Mesh() {
  glDeleteBuffers(1, &vbo);
  if (nbo) {
    glDeleteBuffers(1, &nbo);
  }
  if (cbo) {
    glDeleteBuffers(1, &cbo);
  }
  if (tbo) {
    glDeleteBuffers(1, &tbo);
  }

  glDeleteBuffers(1, &ebo);
  glDeleteVertexArrays(1, &vao);
}

void Mesh::set_texture(const std::shared_ptr<Texture>& texture, GLenum texture_target) {
  this->texture = texture;
  this->texture_target = texture_target;
}

void Mesh::draw(glk::GLSLShader& shader) const {
  if (texture) {
    texture->bind(texture_target);
  }

  const bool cull_was_enabled = glIsEnabled(GL_CULL_FACE);
  if (wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);
    glLineWidth(line_width);
  }

  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, vertex_stride, 0);

  GLint normal_loc = -1;
  if (nbo) {
    normal_loc = shader.attrib("vert_normal");
    glEnableVertexAttribArray(normal_loc);
    glBindBuffer(GL_ARRAY_BUFFER, nbo);
    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE, normal_stride, 0);
  }

  GLint color_loc = -1;
  if (cbo) {
    color_loc = shader.attrib("vert_color");
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, color_stride, 0);
  }

  GLint tex_coord_loc = -1;
  if (tbo) {
    tex_coord_loc = shader.attrib("vert_texcoord");
    glEnableVertexAttribArray(tex_coord_loc);
    glBindBuffer(GL_ARRAY_BUFFER, tbo);
    glVertexAttribPointer(tex_coord_loc, 2, GL_FLOAT, GL_FALSE, tex_coord_stride, 0);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(position_loc);

  if (nbo) {
    glDisableVertexAttribArray(normal_loc);
  }

  if (cbo) {
    glDisableVertexAttribArray(color_loc);
  }

  if (tbo) {
    glDisableVertexAttribArray(normal_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  if (wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (cull_was_enabled) {
      glEnable(GL_CULL_FACE);
    }
    glLineWidth(1.0f);
  }

  if (texture) {
    texture->unbind(texture_target);
  }
}

}  // namespace glk