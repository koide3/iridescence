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
  ThinLines(const float* vertices, int num_vertices, bool line_strip = false);
  ThinLines(const float* vertices, const float* colors, int num_vertices, bool line_strip = false);
  ThinLines(const float* vertices, const float* colors, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);

  template <template <class> class Allocator>
  ThinLines(const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices, bool line_strip = false);

  template <template <class> class Allocator>
  ThinLines(
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>& colors,
    bool line_strip = false);

  template <template <class> class Allocator>
  ThinLines(const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices, const std::vector<unsigned int>& indices, bool line_strip = false);

  template <template <class> class Allocator>
  ThinLines(
    const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
    const std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>& colors,
    const std::vector<unsigned int>& indices,
    bool line_strip = false);

  virtual ~ThinLines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

  void set_line_width(float width) { line_width = width; }

private:
  ThinLines(const ThinLines&);
  ThinLines& operator=(const ThinLines&);

private:
  float line_width;
  int num_vertices;
  int num_indices;

  GLenum mode;  // line mode (GL_LINES/GL_LINE_STRIP)
  GLuint vao;   // vertex array object
  GLuint vbo;   // vertices
  GLuint cbo;   // colors
  GLuint ebo;   // indices
};

// template members

template <template <class> class Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices, bool line_strip) : ThinLines(vertices.front().data(), vertices.size(), line_strip) {}

template <template <class> class Allocator>
ThinLines::ThinLines(
  const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
  const std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>& colors,
  bool line_strip)
: ThinLines(vertices.front().data(), colors.front().data(), vertices.size(), line_strip) {}

template <template <class> class Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices, const std::vector<unsigned int>& indices, bool line_strip)
: ThinLines(vertices.front().data(), nullptr, vertices.size(), indices.data(), indices.size(), line_strip) {}

template <template <class> class Allocator>
ThinLines::ThinLines(
  const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices,
  const std::vector<Eigen::Vector4f, Allocator<Eigen::Vector4f>>& colors,
  const std::vector<unsigned int>& indices,
  bool line_strip)
: ThinLines(vertices.front().data(), colors.front().data(), vertices.size(), indices.data(), indices.size(), line_strip) {}

}  // namespace glk

#endif