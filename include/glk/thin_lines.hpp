#ifndef GLK_THIN_LINES_HPP
#define GLK_THIN_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawble.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

/**
 * @brief A class to draw a set of GL_LINES
 *
 */
class ThinLines : public Drawable {
public:
  ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices);
  ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors);
  virtual ~ThinLines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  ThinLines(const ThinLines&);
  ThinLines& operator=(const ThinLines&);

private:
  int num_vertices;

  GLuint vao;  // vertex array object
  GLuint vbo;  // vertices
  GLuint cbo;  // colors
};
}  // namespace glk

#endif