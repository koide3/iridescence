#ifndef GLK_THIN_LINES_HPP
#define GLK_THIN_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

/**
 * @brief A class to draw a set of GL_LINES
 *
 */
class ThinLines : public Drawable {
public:
  ThinLines(const float* vertices, int num_vertices, bool line_strip=false);
  ThinLines(const float* vertices, const float* colors, int num_vertices, bool line_strip=false);
  ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, bool line_strip=false);
  ThinLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors, bool line_strip=false);
  virtual ~ThinLines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

  void set_line_width(float width) {
    line_width = width;
  }

private:
  ThinLines(const ThinLines&);
  ThinLines& operator=(const ThinLines&);

private:
  float line_width;
  int num_vertices;

  GLenum mode; // line mode (GL_LINES/GL_LINE_STRIP)
  GLuint vao;  // vertex array object
  GLuint vbo;  // vertices
  GLuint cbo;  // colors
};
}  // namespace glk

#endif