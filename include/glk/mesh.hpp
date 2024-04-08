#ifndef GLK_MESH_HPP
#define GLK_MESH_HPP

#include <memory>
#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

class Texture;

class Mesh : public Drawable {
public:
  Mesh(const void* vertices, int vertex_stride, const void* normals, int normal_stride, int num_vertices, const void* indices, int num_indices, bool wireframe = false);
  Mesh(const void* vertices, int vertex_stride, const void* normals, int normal_stride, const void* colors, int color_stride, int num_vertices, const void* indices, int num_indices, bool wireframe = false);
  Mesh(const void* vertices, int vertex_stride, const void* normals, int normal_stride, const void* colors, int color_stride, const void* tex_coords, int tex_coord_stride, int num_vertices, const void* indices, int num_indices, bool wireframe = false);

  template <template <class> class Alloc>
  Mesh(
    const std::vector<Eigen::Vector3f, Alloc<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector3f, Alloc<Eigen::Vector3f>>& normals,
    const std::vector<int>& indices,
    bool wireframe = false)
  : Mesh(vertices.data(), sizeof(float) * 3, normals.data(), sizeof(float) * 3, vertices.size(), indices.data(), indices.size(), wireframe) {}

  template <template <class> class Alloc>
  Mesh(
    const std::vector<Eigen::Vector3f, Alloc<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector3f, Alloc<Eigen::Vector3f>>& normals,
    const std::vector<Eigen::Vector4f, Alloc<Eigen::Vector4f>>& colors,
    const std::vector<int>& indices,
    bool wireframe = false)
  : Mesh(vertices.data(), sizeof(float) * 3, normals.data(), sizeof(float) * 3, colors.data(), sizeof(float) * 4, vertices.size(), indices.data(), indices.size(), wireframe) {}

  virtual ~Mesh();

  virtual void draw(glk::GLSLShader& shader) const override;

  void set_texture(const std::shared_ptr<Texture>& texture, GLenum texture_target = GL_TEXTURE1);

private:
  Mesh(const Mesh&);
  Mesh& operator=(const Mesh&);

private:
  float line_width;
  bool wireframe;

  int vertex_stride;
  int normal_stride;
  int color_stride;
  int tex_coord_stride;

  int num_vertices;
  int num_indices;

  GLuint vao;
  GLuint vbo;
  GLuint nbo;
  GLuint cbo;
  GLuint tbo;
  GLuint ebo;

  GLenum texture_target;
  std::shared_ptr<Texture> texture;
};

}  // namespace glk

#endif