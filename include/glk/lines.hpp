#ifndef GLK_LINES_HPP
#define GLK_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

/**
 * @brief A class to draw a set of lines
 *
 */
class Lines : public Drawable {
public:
  template <template <class> class Allocator>
  Lines(
    float line_width,
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>& colors = std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>(),
    const std::vector<Eigen::Vector4i, Allocator<Eigen::Vector4i>>& infos = std::vector<Eigen::Vector4i, Allocator<Eigen::Vector4i>>());
  virtual ~Lines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  Lines(const Lines&);
  Lines& operator=(const Lines&);

private:
  int num_vertices;
  int num_indices;

  GLuint vao;  // vertex array object
  GLuint vbo;  // vertices
  GLuint cbo;  // colors
  GLuint ibo;  // infos
  GLuint ebo;  // elements
};
}  // namespace glk

#endif